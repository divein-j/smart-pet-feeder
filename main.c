#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <avr/interrupt.h>

//================================================================
// 전역 변수 및 구조체 선언
//================================================================
struct Time {
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t day;
	uint8_t date;
	uint8_t month;
	uint8_t year;
};

struct LastTime {
	uint8_t feed_hour, feed_min, feed_sec;
	uint8_t water_hour, water_min, water_sec;
	} last_time = {0};

	volatile uint8_t feed_hour_1 = 06, feed_min_1 = 00;
	volatile uint8_t feed_hour_2 = 18, feed_min_2 = 00;
	volatile uint8_t emergency_stop = 0;
	volatile uint8_t system_initialized = 0;
	volatile uint32_t water_check_timer = 0;
	volatile uint8_t water_pump_active = 0;
	volatile uint8_t water_pump_remaining = 0;

	#define BAUD 9600
	#define MYUBRR (F_CPU/16/BAUD-1)

	// 물 체크 간격 설정 (10ms 단위)
	#define WATER_FIRST_CHECK 1000     // 10초 후 첫 체크
	#define WATER_CHECK_INTERVAL 60000 // 이후 10분마다 체크

	char uart_buffer[64];
	volatile uint8_t uart_index = 0;
	volatile uint8_t command_ready = 0;

	enum motor_state_t {
		STATE_IDLE,
		STATE_MOVING_OUT,
		STATE_JIGGLE,
		STATE_PAUSED,
		STATE_MOVING_BACK,
		STATE_PAUSED_BETWEEN
	};

	volatile enum motor_state_t g_motor_state = STATE_IDLE;
	volatile uint8_t g_current_angle = 0;
	volatile uint16_t g_pause_counter = 0;
	volatile uint8_t g_speed_counter = 0;
	volatile uint8_t g_feed_cycle_count = 0;

	// 지글 관련 변수
	volatile uint8_t jiggle_count = 0;
	volatile int8_t jiggle_direction = 1;

	const uint8_t MOVE_SPEED = 1;
	const uint16_t PAUSE_DURATION = 50;
	const uint8_t JIGGLE_START_ANGLE = 120;
	const uint8_t JIGGLE_END_ANGLE   = 135;
	const uint8_t JIGGLE_HALF_CYCLES = 6;

	//================================================================
	// 함수 선언부
	//================================================================
	void uart_init(void);
	void uart_transmit(unsigned char data);
	void uart_string(const char* str);
	void i2c_init(void);
	void lcd_init(void);
	void lcd_clear(void);
	void lcd_gotoxy(uint8_t x, uint8_t y);
	void lcd_string(const char* str);
	struct Time ds3231_get_time(void);
	uint8_t is_feed_time(struct Time current);
	void get_next_feed_time(struct Time current, uint8_t* next_hour, uint8_t* next_min);
	uint8_t check_food_level(void);
	uint8_t check_water_level(void);
	void sensor_init(void);
	void water_pump_init(void);
	void water_pump_on(void);
	void water_pump_off(void);
	void water_pump_operation_10sec(void);
	void servo_hw_init(void);
	void servo_set_angle(uint8_t motor_pin, uint8_t angle);
	void servo_feed_operation_start(void);
	void update_servo_motion(void);
	void send_status_bt(void);
	void send_history_bt(void);
	void process_bluetooth_command(char* command);
	void ds3231_set_time(struct Time t);

	//================================================================
	// HC-05 블루투스 통신 함수들
	//================================================================
	void uart_init(void) {
		UBRR1H = (unsigned char)(MYUBRR>>8);
		UBRR1L = (unsigned char)MYUBRR;
		UCSR1B = (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1);
		UCSR1C = (1<<UCSZ11)|(1<<UCSZ10);
	}

	void uart_transmit(unsigned char data) {
		while (!(UCSR1A & (1<<UDRE1)));
		UDR1 = data;
	}

	void uart_string(const char* str) {
		while (*str) {
			uart_transmit(*str++);
		}
	}

	ISR(USART1_RX_vect) {
		char received = UDR1;
		if (received == '\n' || received == '\r') {
			if (uart_index > 0) {
				uart_buffer[uart_index] = '\0';
				command_ready = 1;
				uart_index = 0;
			}
			} else if (uart_index < (sizeof(uart_buffer) - 1)) {
			uart_buffer[uart_index++] = received;
		}
	}

	//================================================================
	// 블루투스 명령 처리 함수들
	//================================================================
	void send_status_bt(void) {
		char status_msg[80];
		struct Time t = ds3231_get_time();
		uint8_t food_level = check_food_level();
		uint8_t water_level = check_water_level();
		
		sprintf(status_msg, "STATUS:%02d:%02d:%02d:%d:%d:%02d:%02d:%02d:%02d\n",
		t.hour, t.min, t.sec, food_level, water_level,
		feed_hour_1, feed_min_1, feed_hour_2, feed_min_2);
		uart_string(status_msg);
	}

	void send_history_bt(void) {
		char history_msg[50];
		sprintf(history_msg, "HISTORY:%02d:%02d:%02d:%02d:%02d:%02d\n",
		last_time.feed_hour, last_time.feed_min, last_time.feed_sec,
		last_time.water_hour, last_time.water_min, last_time.water_sec);
		uart_string(history_msg);
	}

	void process_bluetooth_command(char* command) {
		char command_copy[64];
		strcpy(command_copy, command);

		if (strstr(command_copy, "MANUAL_FEED")) {
			if (emergency_stop || g_motor_state != STATE_IDLE || water_pump_active) {
				uart_string("ERROR:MOTOR_BUSY_OR_STOPPED\n");
				return;
			}
			lcd_clear(); lcd_gotoxy(0, 0); lcd_string("BT MANUAL FEED");
			lcd_gotoxy(0, 1); lcd_string("Fast Dispensing");
			servo_feed_operation_start();
		}
		else if (strstr(command_copy, "MANUAL_WATER")) {
			if (emergency_stop) { uart_string("ERROR:EMERGENCY_STOP\n"); return; }
			if (g_motor_state != STATE_IDLE) { uart_string("ERROR:MOTOR_BUSY\n"); return; }
			if (water_pump_active) { uart_string("ERROR:PUMP_ALREADY_RUNNING\n"); return; }
			
			if (water_pump_remaining == 0) {
				water_pump_remaining = 5;
				lcd_clear(); lcd_gotoxy(0, 0); lcd_string("BT MANUAL WATER");
				} else {
				lcd_clear(); lcd_gotoxy(0, 0); lcd_string("RESUME WATER");
				char resume_msg[30];
				sprintf(resume_msg, "From %d/5\n", 6 - water_pump_remaining);
				uart_string(resume_msg);
			}
			lcd_gotoxy(0, 1); lcd_string("Pumping...");
			
			water_pump_operation_10sec();
			struct Time t = ds3231_get_time();
			last_time.water_hour = t.hour; last_time.water_min = t.min; last_time.water_sec = t.sec;
			uart_string("MANUAL_WATER_OK\n");
		}
		else if (strstr(command_copy, "GET_STATUS")) {
			send_status_bt();
		}
		else if (strstr(command_copy, "GET_HISTORY")) {
			send_history_bt();
		}
		else if (strstr(command_copy, "SET_FEED:")) {
			char* params = command_copy + 9;
			char* token = strtok(params, ":");
			if (token) {
				uint8_t feed_num = atoi(token);
				token = strtok(NULL, ":");
				if (token) {
					uint8_t hour = atoi(token);
					token = strtok(NULL, ":");
					if (token) {
						uint8_t min = atoi(token);
						if (hour <= 23 && min <= 59) {
							if (feed_num == 1) { feed_hour_1 = hour; feed_min_1 = min; }
							else if (feed_num == 2) { feed_hour_2 = hour; feed_min_2 = min; }
							uart_string("SET_FEED_OK\n");
							} else { uart_string("ERROR:INVALID_TIME\n"); }
							} else { uart_string("ERROR:INVALID_FORMAT\n"); }
							} else { uart_string("ERROR:INVALID_FORMAT\n"); }
							} else { uart_string("ERROR:INVALID_FORMAT\n"); }
						}
						else if (strstr(command_copy, "EMERGENCY_STOP")) {
							emergency_stop = 1;
							g_motor_state = STATE_IDLE;
							water_pump_off();
							TCCR1A &= ~((1 << COM1A1) | (1 << COM1C1));
							if (!water_pump_active) {
								lcd_clear(); lcd_gotoxy(0, 0); lcd_string("EMERGENCY STOP!");
								lcd_gotoxy(0, 1); lcd_string("All Motors OFF");
							}
							uart_string("EMERGENCY_STOP_OK\n");
						}
						else if (strstr(command_copy, "EMERGENCY_RESET")) {
							emergency_stop = 0;
							TCCR1A |= (1 << COM1A1) | (1 << COM1C1);
							lcd_clear(); lcd_gotoxy(0, 0); lcd_string("System Reset");
							lcd_gotoxy(0, 1); lcd_string("Normal Mode");
							uart_string("EMERGENCY_RESET_OK\n");
							_delay_ms(2000);
							lcd_clear();
						}
					}

					//================================================================
					// I2C, LCD, RTC 라이브러리
					//================================================================
					#define SCL_CLOCK 100000L
					#define LCD_ADDRESS 0x27
					#define DS3231_ADDRESS 0x68

					void i2c_init(void) { TWSR = 0x00; TWBR = ((F_CPU / SCL_CLOCK) - 16) / 2; }
					void i2c_start(void) { TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); while (!(TWCR & (1 << TWINT))); }
					void i2c_stop(void) { TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); }
					void i2c_write(uint8_t data) { TWDR = data; TWCR = (1 << TWINT) | (1 << TWEN); while (!(TWCR & (1 << TWINT))); }
					uint8_t i2c_read_ack(void) { TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); while (!(TWCR & (1 << TWINT))); return TWDR; }
					uint8_t i2c_read_nack(void) { TWCR = (1 << TWINT) | (1 << TWEN); while (!(TWCR & (1 << TWINT))); return TWDR; }

					void lcd_send(uint8_t data, uint8_t rs) {
						uint8_t high_nibble = data & 0xF0; uint8_t low_nibble = (data << 4) & 0xF0;
						i2c_start(); i2c_write((LCD_ADDRESS << 1) | 0);
						i2c_write(high_nibble | rs | (1 << 2) | (1 << 3)); _delay_us(1);
						i2c_write(high_nibble | rs | (0 << 2) | (1 << 3)); _delay_us(50);
						i2c_write(low_nibble | rs | (1 << 2) | (1 << 3)); _delay_us(1);
						i2c_write(low_nibble | rs | (0 << 2) | (1 << 3));
						i2c_stop(); _delay_ms(2);
					}

					void lcd_send_cmd(uint8_t cmd) { lcd_send(cmd, 0); }
					void lcd_send_data(uint8_t data) { lcd_send(data, 1); }
					void lcd_init(void) { _delay_ms(50); lcd_send_cmd(0x02); lcd_send_cmd(0x28); lcd_send_cmd(0x0C); lcd_send_cmd(0x06); lcd_send_cmd(0x01); _delay_ms(2); }
					void lcd_string(const char *str) { while (*str) lcd_send_data(*str++); }
					void lcd_gotoxy(uint8_t x, uint8_t y) { lcd_send_cmd((y == 0 ? 0x80 : 0xC0) + x); }
					void lcd_clear(void) { lcd_send_cmd(0x01); _delay_ms(2); }

					uint8_t bcd_to_dec(uint8_t val) { return (val / 16 * 10) + (val % 16); }
					uint8_t dec_to_bcd(uint8_t val) { return (val / 10 * 16) + (val % 10); }

					void ds3231_set_time(struct Time t) {
						i2c_start(); i2c_write((DS3231_ADDRESS << 1) | 0); i2c_write(0x00);
						i2c_write(dec_to_bcd(t.sec)); i2c_write(dec_to_bcd(t.min)); i2c_write(dec_to_bcd(t.hour));
						i2c_write(dec_to_bcd(t.day)); i2c_write(dec_to_bcd(t.date)); i2c_write(dec_to_bcd(t.month));
						i2c_write(dec_to_bcd(t.year)); i2c_stop();
					}

					struct Time ds3231_get_time(void) {
						struct Time t;
						i2c_start(); i2c_write((DS3231_ADDRESS << 1) | 0); i2c_write(0x00); i2c_stop();
						i2c_start(); i2c_write((DS3231_ADDRESS << 1) | 1);
						t.sec = bcd_to_dec(i2c_read_ack()); t.min = bcd_to_dec(i2c_read_ack()); t.hour = bcd_to_dec(i2c_read_ack());
						t.day = bcd_to_dec(i2c_read_ack()); t.date = bcd_to_dec(i2c_read_ack()); t.month = bcd_to_dec(i2c_read_ack());
						t.year = bcd_to_dec(i2c_read_nack()); i2c_stop();
						return t;
					}

					//================================================================
					// 배급 시간, 센서, 펌프 함수
					//================================================================
					void get_next_feed_time(struct Time current, uint8_t* next_hour, uint8_t* next_min) {
						uint16_t current_minutes = current.hour * 60 + current.min;
						uint16_t feed1_minutes = feed_hour_1 * 60 + feed_min_1;
						uint16_t feed2_minutes = feed_hour_2 * 60 + feed_min_2;
						if (current_minutes < feed1_minutes) { *next_hour = feed_hour_1; *next_min = feed_min_1; }
						else if (current_minutes < feed2_minutes) { *next_hour = feed_hour_2; *next_min = feed_min_2; }
						else { *next_hour = feed_hour_1; *next_min = feed_min_1; }
					}

					uint8_t is_feed_time(struct Time current) {
						return ((current.hour == feed_hour_1 && current.min == feed_min_1 && current.sec == 0) ||
						(current.hour == feed_hour_2 && current.min == feed_min_2 && current.sec == 0));
					}

					void sensor_init(void) {
						DDRA &= ~((1 << PA0) | (1 << PA1));
						PORTA |= (1 << PA0) | (1 << PA1);
					}

					uint8_t check_food_level(void) {
						return (PINA & (1 << PA0)) ? 1 : 0;
					}

					uint8_t check_water_level(void) {
						uint8_t samples = 0;
						for (uint8_t i = 0; i < 5; i++) {
							if (PINA & (1 << PA1)) samples++;
							_delay_us(100);
						}
						return (samples >= 3) ? 1 : 0;
					}

					void water_pump_init(void) {
						DDRC |= (1 << DDC0);
						PORTC &= ~(1 << PC0);
					}

					void water_pump_on(void) {
						if (!emergency_stop && g_motor_state == STATE_IDLE) {
							PORTC |= (1 << PC0);
						}
					}

					void water_pump_off(void) {
						PORTC &= ~(1 << PC0);
					}

					//================================================================
					// *** 수정된 워터펌프 함수: 서보 완전 차단 후 재초기화 ***
					//================================================================
					void water_pump_operation_10sec(void) {
						if (emergency_stop || g_motor_state != STATE_IDLE) {
							uart_string("ERROR:PUMP_BLOCKED\n");
							return;
						}
						
						uart_string("PUMP_START:DISABLING_SERVO\n");
						water_pump_active = 1;
						
						// *** 핵심 수정: 서보모터 PWM 완전 차단 ***
						TCCR1A = 0;  // 타이머1 비교출력 완전 OFF
						TCCR1B = 0;  // 타이머1 클럭 소스 OFF
						DDRB &= ~((1 << DDB5) | (1 << DDB7));  // 핀을 입력으로 전환
						_delay_ms(100);  // 안정화 대기
						uart_string("SERVO_DISABLED\n");
						
						// 워터펌프 작동
						lcd_clear();
						lcd_gotoxy(0, 0);
						lcd_string("WATER PUMPING");
						lcd_gotoxy(0, 1);
						lcd_string("Please wait...");
						
						water_pump_on();
						uart_string("PUMP_ON\n");
						
						uint8_t start_sec = 5 - water_pump_remaining;
						for (uint8_t i = 0; i < water_pump_remaining; i++) {
							for (uint8_t j = 0; j < 100; j++) {
								_delay_ms(10);
								
								if (command_ready) {
									if (strstr(uart_buffer, "EMERGENCY_STOP")) {
										water_pump_remaining = water_pump_remaining - i;
										process_bluetooth_command(uart_buffer);
									}
									command_ready = 0;
								}
								
								if (emergency_stop) {
									goto pump_exit;
								}
							}
							
							if (i % 2 == 0) {
								char progress[20];
								sprintf(progress, "PUMP:%d/5\n", start_sec + i + 1);
								uart_string(progress);
							}
						}
						
						water_pump_remaining = 0;
						
						pump_exit:
						water_pump_off();
						uart_string("\nPUMP_OFF\n");
						water_pump_active = 0;
						
						if (emergency_stop) {
							lcd_clear();
							lcd_gotoxy(0, 0);
							lcd_string("EMERGENCY STOP!");
							lcd_gotoxy(0, 1);
							lcd_string("All Motors OFF");
							_delay_ms(2000);
						}
						
						// *** 펌프 OFF 후 물 안정화 대기 시간 ***
						uart_string("WAITING_FOR_WATER_SETTLE\n");
						_delay_ms(2000);
						
						// *** 핵심 수정: 서보모터 재초기화 ***
						uart_string("RE-INITIALIZING_SERVO\n");
						DDRB |= (1 << DDB5) | (1 << DDB7);  // 핀을 출력으로 재설정
						TCCR1A = (1 << COM1A1) | (1 << COM1C1) | (1 << WGM11);  // 비교출력 활성화
						TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);     // Fast PWM, 분주비 8
						ICR1 = 39999;   // TOP 값 설정 (50Hz)
						OCR1A = 2000;   // 0도 위치
						OCR1C = 2000;   // 0도 위치
						g_current_angle = 0;  // 각도 초기화
						_delay_ms(50);
						uart_string("SERVO_RESTORED:PUMP_COMPLETE\n");
					}

					//================================================================
					// 듀얼 하드웨어 PWM 서보모터 제어 (PB5, PB7)
					//================================================================
					void servo_hw_init(void) {
						DDRB |= (1 << DDB5) | (1 << DDB7);
						TCCR1A = (1 << COM1A1) | (1 << COM1C1) | (1 << WGM11);
						TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
						ICR1 = 39999;
						OCR1A = 2000;
						OCR1C = 2000;
						g_current_angle = 0;
						uart_string("SERVO_FAST_MODE_INIT\n");
					}

					void servo_set_angle(uint8_t motor_pin, uint8_t angle) {
						if (emergency_stop) return;
						uint16_t ocr_value = 2000 + ((uint32_t)angle * 2000) / 180;
						if (motor_pin == 5) OCR1A = ocr_value;
						else if (motor_pin == 7) OCR1C = ocr_value;
					}

					void servo_feed_operation_start(void) {
						if (g_motor_state == STATE_IDLE && !emergency_stop) {
							uart_string("SERVO_MOTION_START\n");
							g_motor_state = STATE_MOVING_OUT;
							g_current_angle = 0;
						}
					}

					//================================================================
					// 서보 동작 상태머신 (단순 버전)
					//================================================================
					void update_servo_motion(void) {
						if (g_motor_state == STATE_IDLE || emergency_stop) {
							return;
						}
						
						g_speed_counter++;
						if (g_speed_counter < MOVE_SPEED) {
							return;
						}
						g_speed_counter = 0;

						switch (g_motor_state) {
							case STATE_MOVING_OUT:
							if (g_current_angle < 135) {
								g_current_angle++;
								servo_set_angle(5, g_current_angle);
								servo_set_angle(7, g_current_angle);
								if (g_current_angle % 10 == 0) {
									char progress[20];
									sprintf(progress, "ANGLE:%d\n", g_current_angle);
									uart_string(progress);
								}
								} else {
								g_motor_state = STATE_PAUSED;
								g_pause_counter = 0;
								uart_string("REACHED_135_DEGREES\n");
							}
							break;

							case STATE_PAUSED:
							g_pause_counter++;
							if (g_pause_counter >= PAUSE_DURATION) {
								g_motor_state = STATE_MOVING_BACK;
								uart_string("RETURNING_TO_0\n");
							}
							break;

							case STATE_MOVING_BACK:
							if (g_current_angle > 0) {
								g_current_angle--;
								servo_set_angle(5, g_current_angle);
								servo_set_angle(7, g_current_angle);
								if (g_current_angle % 10 == 0) {
									char progress[20];
									sprintf(progress, "RETURN:%d\n", g_current_angle);
									uart_string(progress);
								}
								} else {
								g_motor_state = STATE_IDLE;
								uart_string("SERVO_MOTION_COMPLETE\n");

								struct Time t = ds3231_get_time();
								last_time.feed_hour = t.hour;
								last_time.feed_min = t.min;
								last_time.feed_sec = t.sec;
							}
							break;

							case STATE_IDLE:
							default:
							break;
						}
					}

					//================================================================
					// 메인 함수
					//================================================================
					int main(void)
					{
						i2c_init();
						lcd_init();
						uart_init();
						sensor_init();
						water_pump_init();
						_delay_ms(100);
						servo_hw_init();
						sei();
						
						struct Time t;
						char time_str[16], date_str[16], next_feed_str[20];
						uint8_t next_hour, next_min, display_mode = 0, feeding_done = 0;
						uint16_t loop_counter = 0;
						uint8_t water_check_done = 0;

						water_pump_off();
						_delay_ms(100);

						lcd_clear();
						lcd_gotoxy(0, 0);
						lcd_string("Smart Pet Feeder");
						_delay_ms(2000);
						
						lcd_clear();
						lcd_gotoxy(0, 0); lcd_string("Sensor Check...");
						lcd_gotoxy(0, 1); lcd_string("Please wait");
						
						for (uint8_t i = 0; i < 5; i++) {
							check_water_level();
							check_food_level();
							_delay_ms(100);
						}
						_delay_ms(1500);
						
						lcd_clear();
						lcd_gotoxy(0, 0); lcd_string("Food:");
						lcd_string(check_food_level() ? "OK" : "Low");
						lcd_gotoxy(0, 1); lcd_string("Water:");
						lcd_string(check_water_level() ? "OK" : "Low");
						_delay_ms(2000);
						
						lcd_clear();
						lcd_gotoxy(0, 0);
						lcd_string("System Ready!");
						_delay_ms(1000);

						system_initialized = 1;
						water_check_timer = 0;
						display_mode = 0;
						loop_counter = 1;
						uart_string("SYSTEM_READY_V4.3_SIMPLE_MOTION\n");

						// 첫 화면 즉시 표시
						lcd_clear();
						t = ds3231_get_time();
						sprintf(time_str, "%02d:%02d:%02d", t.hour, t.min, t.sec);
						sprintf(date_str, "20%02d-%02d-%02d", t.year, t.month, t.date);
						lcd_gotoxy(0, 0);
						lcd_string(date_str);
						lcd_gotoxy(0, 1);
						lcd_string(time_str);

						while (1)
						{
							if (command_ready) {
								process_bluetooth_command(uart_buffer);
								command_ready = 0;
							}
							
							update_servo_motion();

							// 매 루프마다 타이머 증가 (10ms마다)
							if (system_initialized && !emergency_stop) {
								water_check_timer++;
							}

							// 100 루프마다 실행 (1초마다)
							if (loop_counter % 100 == 0) {
								if (!emergency_stop && !water_pump_active) {
									t = ds3231_get_time();

									// 급식 체크
									if (is_feed_time(t) && !feeding_done && g_motor_state == STATE_IDLE) {
										if (!check_food_level()) {
											lcd_clear();
											lcd_gotoxy(0, 0);
											lcd_string("FEEDING TIME!");
											lcd_gotoxy(0, 1);
											lcd_string("                ");
											uart_string("AUTO_FEED_FAST_START\n");
											servo_feed_operation_start();
											} else {
											lcd_clear();
											lcd_gotoxy(0, 0);
											lcd_string("ENOUGH FOOD!");
											lcd_gotoxy(0, 1);
											lcd_string("                ");
											_delay_ms(3000);
										}
										feeding_done = 1;
										} else if (!is_feed_time(t)) {
										feeding_done = 0;
									}
									
									// LCD 업데이트
									if (g_motor_state == STATE_IDLE) {
										get_next_feed_time(t, &next_hour, &next_min);
										sprintf(time_str, "%02d:%02d:%02d", t.hour, t.min, t.sec);
										sprintf(date_str, "20%02d-%02d-%02d", t.year, t.month, t.date);
										sprintf(next_feed_str, "Next: %02d:%02d", next_hour, next_min);
										
										switch (display_mode) {
											case 0:
											lcd_gotoxy(0, 0);
											lcd_string(date_str);
											lcd_string("  ");
											lcd_gotoxy(0, 1);
											lcd_string(time_str);
											lcd_string("       ");
											break;
											
											case 1: {
												char schedule[17];
												sprintf(schedule, "%02d:%02d & %02d:%02d",
												feed_hour_1, feed_min_1, feed_hour_2, feed_min_2);
												lcd_gotoxy(0, 0);
												lcd_string("Feeding Times   ");
												lcd_gotoxy(0, 1);
												lcd_string(schedule);
												break;
											}
											
											case 2:
											lcd_gotoxy(0, 0);
											lcd_string("Food: ");
											lcd_string(check_food_level() ? "Available " : "Empty!    ");
											lcd_gotoxy(0, 1);
											lcd_string("                ");
											break;
											
											case 3:
											lcd_gotoxy(0, 0);
											lcd_string("Water: ");
											lcd_string(check_water_level() ? "Available " : "Empty!    ");
											lcd_gotoxy(0, 1);
											lcd_string("                ");
											break;
										}
									}
								}
							}
							
							// 400 루프마다 디스플레이 모드 변경 (4초마다)
							if (loop_counter % 400 == 0) {
								if (!emergency_stop && !water_pump_active) {
									display_mode = (display_mode + 1) % 4;
									if(g_motor_state == STATE_IDLE) lcd_clear();
								}
							}
							
							// 자동 물 공급 체크
							if (system_initialized && !emergency_stop && g_motor_state == STATE_IDLE && !water_pump_active) {
								// 첫 번째 체크: 10초 후
								if (water_check_timer == WATER_FIRST_CHECK && !water_check_done) {
									uart_string("FIRST_WATER_CHECK:10SEC\n");
									
									uint8_t water_before = check_water_level();
									char before_msg[30];
									sprintf(before_msg, "WATER_BEFORE:%d\n", water_before);
									uart_string(before_msg);
									
									if (water_before == 0) {
										uart_string("WATER_LOW:PUMPING\n");
										water_pump_remaining = 5;
										water_pump_operation_10sec();
										
										struct Time wt = ds3231_get_time();
										last_time.water_hour = wt.hour;
										last_time.water_min = wt.min;
										last_time.water_sec = wt.sec;
										
										uart_string("CHECKING_AFTER_PUMP\n");
										_delay_ms(1000);
										
										uint8_t water_after = check_water_level();
										char after_msg[30];
										sprintf(after_msg, "WATER_AFTER:%d\n", water_after);
										uart_string(after_msg);
										
										if (water_after == 1) {
											lcd_clear();
											lcd_gotoxy(0, 0);
											lcd_string("Water Refilled");
											lcd_gotoxy(0, 1);
											lcd_string("Complete!");
											uart_string("WATER_REFILL_SUCCESS\n");
											} else {
											lcd_clear();
											lcd_gotoxy(0, 0);
											lcd_string("Water Error!");
											lcd_gotoxy(0, 1);
											lcd_string("Check System");
											uart_string("WATER_REFILL_FAILED\n");
										}
										_delay_ms(2000);
										lcd_clear();
										} else {
										uart_string("WATER_OK:NO_PUMP_NEEDED\n");
									}
									water_check_done = 1;
								}
								
								// 이후 체크: 10분마다
								if (water_check_done && water_check_timer >= WATER_FIRST_CHECK) {
									uint32_t elapsed_since_first = water_check_timer - WATER_FIRST_CHECK;
									
									if (elapsed_since_first > 0 && (elapsed_since_first % WATER_CHECK_INTERVAL == 0)) {
										char timer_msg[40];
										sprintf(timer_msg, "PERIODIC_CHECK:T=%lu\n", water_check_timer);
										uart_string(timer_msg);
										
										uint8_t water_before = check_water_level();
										char before_msg[30];
										sprintf(before_msg, "WATER_BEFORE:%d\n", water_before);
										uart_string(before_msg);
										
										if (water_before == 0) {
											uart_string("WATER_LOW:PUMPING\n");
											water_pump_remaining = 5;
											water_pump_operation_10sec();
											
											struct Time wt = ds3231_get_time();
											last_time.water_hour = wt.hour;
											last_time.water_min = wt.min;
											last_time.water_sec = wt.sec;
											
											uart_string("CHECKING_AFTER_PUMP\n");
											_delay_ms(1000);
											
											uint8_t water_after = check_water_level();
											char after_msg[30];
											sprintf(after_msg, "WATER_AFTER:%d\n", water_after);
											uart_string(after_msg);
											
											if (water_after == 1) {
												lcd_clear();
												lcd_gotoxy(0, 0);
												lcd_string("Water Refilled");
												lcd_gotoxy(0, 1);
												lcd_string("Complete!");
												uart_string("WATER_REFILL_SUCCESS\n");
												} else {
												lcd_clear();
												lcd_gotoxy(0, 0);
												lcd_string("Water Error!");
												lcd_gotoxy(0, 1);
												lcd_string("Check System");
												uart_string("WATER_REFILL_FAILED\n");
											}
											_delay_ms(2000);
											lcd_clear();
											} else {
											uart_string("WATER_OK:NO_PUMP_NEEDED\n");
										}
									}
								}
							}
							
							// 상태 전송 (60초마다)
							if (loop_counter >= 6000) {
								if (!emergency_stop) {
									send_status_bt();
								}
								loop_counter = 0;
							}

							loop_counter++;
							_delay_ms(10);
						}
					}
