#define F_CPU 14745600
	#include <avr/io.h>
	#include <avr/interrupt.h>
	#include <util/delay.h>
	#include "lcd.h"

    volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
	volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
	volatile unsigned int Degrees;
	//to accept angle in degrees for turning
	int count = 0 ;	

	
	//info_table contains two information : first columns represent availability of block and second column represent sequence number

	int info_table[12][2] = {{1,1},{0,2},{2,3},{0,4},{1,5},{0,6},{3,7},{0,8},{3,9},{0,10},{0,11},{4,12}};  // 0 represent absence of block at that location .
																										   // 1,2,3,4 represent blue,yellow,red,green block
	unsigned char data;
	
	void buzzer_pin_config (void)
	{
	 DDRC = DDRC | 0x08;    //Setting PORTC 3 as output
	 PORTC = PORTC & 0xF7;    //Setting PORTC 3 logic low to turnoff buzzer
	}
	
	//Configure PORTB 5 pin for servo motor 1 operation
	void servo1_pin_config (void)
	{
	 DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
	 PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
	}
	
	//Configure PORTB 6 pin for servo motor 2 operation
	void servo2_pin_config (void)
	{
	 DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
	 PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
	}
	
	
	//Configure PORTB 7 pin for servo motor 3 operation
	void servo3_pin_config (void)
	{
	 DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
	 PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
	}
	
	void port_init();
	void timer5_init();
	void velocity(unsigned char, unsigned char);
	void motors_delay();
	void stop(void);
	void left(void);
	void right(void);
	void soft_left(void);
	void soft_right(void);
	void soft_left_2(void);
	void soft_right_2(void);
	
	unsigned char ADC_Conversion(unsigned char);
	unsigned char ADC_Value;
	unsigned char sharp, distance, adc_reading;
	unsigned int value;
	unsigned char flag = 0;
	unsigned char Left_white_line = 0;
	unsigned char Center_white_line = 0;
	unsigned char Right_white_line = 0;
	
	void left_encoder_pin_config (void)
	{
		DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
		PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
	}
	
	//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
	void right_encoder_pin_config (void)
	{
		DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
		PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
	}
	
	//Function to configure LCD port
	void lcd_port_config (void)
	{
	 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
	}
	
	//ADC pin configuration
	void adc_pin_config (void)
	{
	 DDRF = 0x00;
	 PORTF = 0x00;
	 DDRK = 0x00;
	 PORTK = 0x00;
	}
	
	//Function to configure ports to enable robot's motion
	void motion_pin_config (void)
	{
	 DDRA = DDRA | 0x0F;
	 PORTA = PORTA & 0xF0;
	 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
	}

	void uart0_init(void)
	{
		UCSR0B = 0x00; //disable while setting baud rate
		UCSR0A = 0x20;
		UCSR0C = 0x06;
		UBRR0L = 0x5F; //set baud rate lo
		UBRR0H = 0x00; //set baud rate hi
		UCSR0B = 0x68;
	}

	SIGNAL(SIG_USART0_DATA) 		// ISR for transmit complete interrupt
	{
		
		
		UDR0 = data;
		_delay_ms(50);

		

	}
	SIGNAL(SIG_USART0_TRANS)    
	{
		data = 0x00 ;
	};

	
	//Function to Initialize PORTS
	void port_init()
	{
		buzzer_pin_config();
		servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
	 	servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation
	 	servo3_pin_config(); //Configure PORTB 7 pin for servo motor 3 operation
		
	  	lcd_port_config();
		adc_pin_config();
		motion_pin_config();
		left_encoder_pin_config(); //left encoder pin config
		right_encoder_pin_config(); //right encoder pin config
	}
	
	
	void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
	{
		cli(); //Clears the global interrupt
		EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
		EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
		sei();   // Enables the global interrupt
	}
	
	void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
	{
		cli(); //Clears the global interrupt
		EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
		EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
		sei();   // Enables the global interrupt
	}
	
	ISR(INT5_vect)
	{
		ShaftCountRight++;  //increment right shaft position count
	}
	
	//ISR for left position encoder
	ISR(INT4_vect)
	{
		ShaftCountLeft++;  //increment left shaft position count
	}
	
	// Timer 5 initialized in PWM mode for velocity control
	// Pre-scale:256
	// PWM 8bit fast, TOP=0x00FF
	// Timer Frequency:225.000Hz
	void timer5_init()
	{
		TCCR5B = 0x00;	//Stop
		TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
		TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
		OCR5AH = 0x00;	//Output compare register high value for Left Motor
		OCR5AL = 0xFF;	//Output compare register low value for Left Motor
		OCR5BH = 0x00;	//Output compare register high value for Right Motor
		OCR5BL = 0xFF;	//Output compare register low value for Right Motor
		OCR5CH = 0x00;	//Output compare register high value for Motor C1
		OCR5CL = 0xFF;	//Output compare register low value for Motor C1
		TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
	 					  For Overriding normal port functionality to OCRnA outputs.
					  	  {WGM51=0, WGM80=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/

		TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
	}
	void timer1_init(void)
	{
	 TCCR1B = 0x00; //stop
	 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
	 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
	 OCR1AH = 0x03;	//Output compare Register high value for servo 1
	 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
	 OCR1BH = 0x03;	//Output compare Register high value for servo 2
	 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
	 OCR1CH = 0x03;	//Output compare Register high value for servo 3
	 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
	 ICR1H  = 0x03;
	 ICR1L  = 0xFF;
	 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
	 					For Overriding normal port functionality to OCRnA outputs.
					  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
	 TCCR1C = 0x00;
	 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
	}
	
	void adc_init()
	{
		ADCSRA = 0x00;
		ADCSRB = 0x00;		//MUX5 = 0
		ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
		ACSR = 0x80;
		ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
	}

	//Function For ADC Conversion
	unsigned char ADC_Conversion(unsigned char Ch)
	{
		unsigned char a;
		if(Ch>7)
		{
			ADCSRB = 0x08;
		}
		Ch = Ch & 0x07;
		ADMUX= 0x20| Ch;
		ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
		while((ADCSRA&0x10)==0);	//Wait for conversion to complete
		a=ADCH;
		ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
		ADCSRB = 0x00;
		return a;
	}

	//Function To Print Sensor Values At Desired Row And Column Location on LCD
	void print_sensor(char row, char coloumn,unsigned char channel)
	{

		ADC_Value = ADC_Conversion(channel);
		lcd_print(row, coloumn, ADC_Value, 3);
	}
	
	// This Function calculates the actual distance in millimeters(mm) from the input
	// analog value of Sharp Sensor.
	unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
	{
		float distance;
		unsigned int distanceInt;
		distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
		distanceInt = (int)distance;
		if(distanceInt>800)
		{
			distanceInt=800;
		}
		return distanceInt;
	}
	
	//Function for velocity control
	void velocity (unsigned char left_motor, unsigned char right_motor)
	{
		OCR5AL = (unsigned char)left_motor;
		OCR5BL = (unsigned char)right_motor;
	}
	
	//Function used for setting motor's direction
	void motion_set (unsigned char Direction)
	{
	 unsigned char PortARestore = 0;

	 Direction &= 0x0F; 		// removing upper nibbel for the protection
	 PortARestore = PORTA; 		// reading the PORTA original status
	 PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	 PORTA = PortARestore; 		// executing the command
	}

	void angle_rotate(unsigned int Degrees)
	{
		float ReqdShaftCount = 0;
		unsigned long int ReqdShaftCountInt = 0;

		ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
		ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
		ShaftCountRight = 0;
		ShaftCountLeft = 0;

		while (1)
		{
			if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
			break;
		}
		stop(); //Stop robot
	}
	
	// uses angle_rotate() to turn specific angles to left
	void left_degrees(unsigned int Degrees)
	{
		// 88 pulses for 360 degrees rotation 4.090 degrees per count
		left(); //Turn left
		//velocity(160,160);
		angle_rotate(Degrees);
	}


	// uses angle_rotate() to turn specific angles to left
	void right_degrees(unsigned int Degrees)
	{
		// 88 pulses for 360 degrees rotation 4.090 degrees per count
		right(); //Turn right
		angle_rotate(Degrees);
	}


	void forward (void)
	{
	  motion_set (0x06);
	}

	void back (void) //both wheels backward
	{
	  motion_set(0x09);
	}

	void stop (void)
	{
	  motion_set (0x00);
	}

	void left (void) //Left wheel backward, Right wheel forward
	{
	  motion_set(0x05);
	}

	void right (void) //Left wheel forward, Right wheel backward
	{
	  motion_set(0x0A);
	}

	void soft_left (void) //Left wheel stationary, Right wheel forward
	{
	 motion_set(0x04);
	}

	void soft_right (void) //Left wheel forward, Right wheel is stationary
	{
	 motion_set(0x02);
	}

	void soft_left_2 (void) //Left wheel backward, right wheel stationary
	{
	 motion_set(0x01);
	}

	void soft_right_2 (void) //Left wheel stationary, Right wheel backward
	{
	 motion_set(0x08);
	}
	
	void init_devices (void)
	{
	 	cli(); //Clears the global interrupts
		uart0_init();
		port_init();
		adc_init();
		timer5_init();
		left_position_encoder_interrupt_init();
		right_position_encoder_interrupt_init();
		timer1_init();
		sei();   //Enables the global interrupts
	}
	
	// function to set servo motor on pin 1 to motion (for a specified angle of rotation)
	void servo_1(unsigned char degrees)
	{
	 float PositionPanServo = 0;
	  PositionPanServo = ((float)degrees / 1.86) + 35.0;
	 OCR1AH = 0x00;
	 OCR1AL = (unsigned char) PositionPanServo;
	}


	//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
	void servo_2(unsigned char degrees)
	{
	 float PositionTiltServo = 0;
	 PositionTiltServo = ((float)degrees / 1.86) + 35.0;
	 OCR1BH = 0x00;
	 OCR1BL = (unsigned char) PositionTiltServo;
	}

	//Function to rotate Servo 3 by a specified angle in the multiples of 1.86 degrees
	void servo_3(unsigned char degrees)
	{
	 float PositionServo = 0;
	 PositionServo = ((float)degrees / 1.86) + 35.0;
	 OCR1CH = 0x00;
	 OCR1CL = (unsigned char) PositionServo;
	}

	void servo_1_free (void) //makes servo 1 free rotating
	{
	 OCR1AH = 0x03;
	 OCR1AL = 0xFF; //Servo 1 off
	}

	void servo_2_free (void) //makes servo 2 free rotating
	{
	 OCR1BH = 0x03;
	 OCR1BL = 0xFF; //Servo 2 off
	}

	void servo_3_free (void) //makes servo 3 free rotating
	{
	 OCR1CH = 0x03;
	 OCR1CL = 0xFF; //Servo 3 off
	}
	
	
	
	/*
	 * Function Name: forward_black
	 * Variables: None
	 * Input: Left_white_line, Center_white_line, Right_white_line variable values, which are actually the three white line sensor values
	 	respectively.
	 * Output: Corresponding motion according to the combination defined in logic.
	 * Logic: Sensors can have seven different conditions inside on basis of 7 different combinations of sensor values.
	 	The combination is commented just above each condition as 010, 101, 111, 000 etc.
	 	0 corresponds for a white region and 1 corresponds for a black region.
	 	eg: 010 represents: Left_white_line sensor color : white
	 						Center_white_line sensor color : black
	 						Right_white_line sensor color : white
		A node is defined when all three white line sensors detect black color. 
		0x0c is the threshold sensor value upto which the sensor reads a white color.
	 * Example Call: forward_black();
	 
	 
	 */
	int forward_black()
	{
		while(1){
			Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

			if(Center_white_line>0x20 && Left_white_line<0x20 && Right_white_line<0x20) // Center on black line-010
			{	forward();
				velocity(150,142); //velocity value calibrated as per requirement

			}

			else if((Left_white_line>0x20 && Center_white_line<0x20) ) //left sensor on black line, take left turn to get back on blackline
			{
				forward();
				velocity(40,120); //velocity of left and right motors calibrated as per requirement
				_delay_ms(15);
			}

			else if((Right_white_line>0x20 && Center_white_line<0x20)) //right sensor on black line, take right turn to get back on blackline
			{	forward();
				velocity(120,40);
				_delay_ms(15);
			}
			
			else if(Center_white_line>0x20 && Left_white_line>0x20 && Right_white_line>0x20){//111
				stop();
				//++count;
				return 0;
			}
			

		}

	}

	/*
	 * Function Name: turn_left
	 * Input: Left_white_line,  which is actually the value of leftmost white line sensor.
	 * Output: Helps in taking a restricted left turn on the basis of certain condition that is defined in logic.
	 * Logic: Takes a left until its left sensor gets a black value.
	 	0x0c is the threshold sensor value upto which the sensor reads a white color.
	 * Example Call: turn_left();
	
	 */

	void turn_left()
	{
		left();
		while(1)
		{

			Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

			flag=0;

			print_sensor(1,1,3);	//Prints value of White Line Sensor1
			print_sensor(1,5,2);	//Prints Value of White Line Sensor2
			print_sensor(1,9,1);	//Prints Value of White Line Sensor3

			velocity(110,110);
			_delay_ms(6);
			if(Center_white_line>=0x40)
			{	stop();
				velocity(0,0);
				return;
			}

		}
	}

	/*
	 * Function Name: center_right
	 * Input: Center_white_line,  which is actually the value of center white line sensor.
	 * Output: Rotates the robot to take a restricted right turn.
	 * Logic: Takes a right until its center sensor gets a black value.
	 	0x0c is the threshold sensor value upto which the sensor reads a white color.
	 * Example Call: center_right();

	 */

	void center_right()
	{
		soft_right_2();
		while(1)
		{

			Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

			flag=0;

			print_sensor(1,1,3);	//Prints value of White Line Sensor1
			print_sensor(1,5,2);	//Prints Value of White Line Sensor2
			print_sensor(1,9,1);	//Prints Value of White Line Sensor3

			velocity(0,130);
			_delay_ms(6);
			if(Center_white_line>=0x0b)
			{
				velocity(0,0);

				return;
			}

		}
	}


	/*
	 * Function Name: turn_right
	 * Input: Right_white_line,  which is actually the value of rightmost white line sensor.
	 * Output: Helps in taking a restricted right turn on the basis of certain condition that is defined in logic.
	 * Logic: Takes a right until its right sensor gets a black value.
	 	0x0c is the threshold sensor value upto which the sensor reads a white color.
	 * Example Call: turn_right();
	
	 */

	void turn_right()
	{
		right();
		while(1)
		{

			Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

			flag=0;

			print_sensor(1,1,3);	//Prints value of White Line Sensor1
			print_sensor(1,5,2);	//Prints Value of White Line Sensor2
			print_sensor(1,9,1);	//Prints Value of White Line Sensor3

			velocity(110,110);
			_delay_ms(6);
			if(Center_white_line>=0x40)
			{
				velocity(0,0);

				return;
			}


		}
	}


	// function to set the buzzer on
	
	void buzzer_on (void)
	{
	 unsigned char port_restore = 0;
	 port_restore = PINC;
	 port_restore = port_restore | 0x08;
	 PORTC = port_restore;
	}

	void buzzer_off (void)
	{
	 unsigned char port_restore = 0;
	 port_restore = PINC;
	 port_restore = port_restore & 0xF7;
	 PORTC = port_restore;
	}
	
	
	//Function used for moving robot forward by specified distance

	void linear_distance_mm(unsigned int DistanceInMM)
	{
		float ReqdShaftCount = 0;
		unsigned long int ReqdShaftCountInt = 0;

		ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
		ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;

		ShaftCountRight = 0;
		while(1)
		{
			if(ShaftCountRight > ReqdShaftCountInt)
			{
				break;
			}
		}
		stop(); //Stop robot
	}

	void forward_mm(unsigned int DistanceInMM)
	{
		forward();
		linear_distance_mm(DistanceInMM);
	}

	void back_mm(unsigned int DistanceInMM)
	{
		back();
		linear_distance_mm(DistanceInMM);
	}
	
	
	/*
	? * Function Name:rotation_using_blackline_right
	? * Logic: This function is used to rotate robot right until it finds a blackline.It is used only when robot is at a node.
	          Since robot will be at node it must be moved from it or otherwise turn_right function will keep robot at same position.
			  Thus robot is forwarded and then turn_right function is called. 
	? * Example Call:rotation_using_blackline_right
	? */
	void rotation_using_blackline_right(void)
	{
	forward_mm(89); //robot is forwarded by fixed distance using encoder
	stop();
	_delay_ms(500);


	right_degrees(90);
	_delay_ms(50);
	stop();


	}
	/*
	? * Function Name:rotation_using_blackline_left
	? * Logic: This function is used to rotate robot left until it finds a blakline.It is used only when robot is at a node.
	          Since robot will be at node it must be moved from it or otherwise turn_left function will keep robot at same position.
			  Thus robot is forwarded and then turn_left function is called. 
	? * Example Call: rotation_using_blackline_left();
	? */
	void rotation_using_blackline_left(void)
	{
		forward_mm(89); //robot is forwarded by fixed distance using encoder
		stop();
		_delay_ms(500);
		left_degrees(90);
		_delay_ms(50);
		stop();
	}
	
//Function to count node in path of firebird V. A node is defined as the junction where value of all three black line sensors is geater than threshold value.
int count_node(void) {
			
			Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

			if(Center_white_line>0x20 && Left_white_line>0x20 && Right_white_line>0x20){//111
				stop();
				count++ ;
			
			}
			return count;
}

//function to reset node count value to 0
void reset_count(void){
	count = 0 ;
	return;
}

//function to move robot from node to node
// logic :- If the robot is at a node, then black line following function can not be used. So, this function sets the robot in motion
//  		initially and then does proper black line following till next node is reached
void forward_from_node(void){
			
			forward_mm(30);
			_delay_ms(50);
			while(1){
			
			Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
			if(Center_white_line>0x20 && Left_white_line<0x20 && Right_white_line<0x20) // Center on black line-010
			{	forward();
				velocity(130,122); //velocity value calibrated as per requirement

			}

		    else if((Left_white_line>0x20 && Center_white_line<0x20) ) //left sensor on black line, take left turn to get back on blackline
			{
				forward();
				velocity(40,120); //velocity of left and right motors calibrated as per requirement
				_delay_ms(15);
			}

			else if((Right_white_line>0x20 && Center_white_line<0x20)) //right sensor on black line, take right turn to get back on blackline
			{	forward();
				velocity(120,40);
				_delay_ms(15);
			}
			else if(Center_white_line>0x20 && Left_white_line>0x20 && Right_white_line>0x20){//111
				stop();
				//++count;
				 break;

				}
}
return;
}

//function to move forward 30mm and rotate left robot  until black line found based on flag value
// if flag =0 it will rotate until first black line detected
// if flag =1 it will rotate until second black line detected
void rotation_using_blackline_newleft(unsigned int flag =0)
{
	forward_mm(30); //robot is forwarded by fixed distance using encoder
	stop();
	_delay_ms(500);
	if(flag ==0)
	{
	
		turn_left();     //robot will rotate left until it gets blackline
		velocity(190,190);
		_delay_ms(50);
		stop();
	}
	else {
		
		turn_left();     //robot will rotate left until it gets blackline
		_delay_ms(50);
		turn_left();
		velocity(160,160);
		_delay_ms(50);
		stop();
	}
	
}
//function to move forward 30mm and rotate robot right  until black line found based on flag value
// if flag =0 it will rotate until first black line detected
// if flag =1 it will rotate until second black line detected

void rotation_using_blackline_newright(unsigned int flag =0)
{
	forward_mm(30); //robot is forwarded by fixed distance using encoder
	stop();
	_delay_ms(500);
	if(flag ==0)
	{
			turn_right();     //robot will rotate left until it gets blackline
			velocity(160,160);
			_delay_ms(50);
			stop();
	}
	else {
			turn_right();     //robot will rotate left until it gets blackline
			_delay_ms(50);
			turn_right();
			velocity(160,160);
			_delay_ms(50);
			stop();
	}

}
//function to rotate left robot until black line found based on flag value
// if flag =0 it will rotate until first black line detected
// if flag =1 it will rotate until second black line detected
void rotation_blackline_left(unsigned int flag =0)
{
	if(flag ==0)
	{
		turn_left();
		velocity(160,160);
		_delay_ms(50);
		stop();
		
	}
	else {
		turn_left();
		_delay_ms(50);
		turn_left();
		velocity(160,160);
		_delay_ms(50);
		stop();
		
	}
	
	
}
//function to rotate right robot until black line found based on flag value
// if flag =0 it will rotate until first black line detected
// if flag =1 it will rotate until second black line detected
void rotation_blackline_right(unsigned int flag =0)
{
	if(flag ==0)
	{
		turn_right();
		velocity(160,160);
		_delay_ms(50);
		stop();
		
	}
	else {
		turn_right();
		_delay_ms(50);
		turn_right();
		velocity(160,160);
		_delay_ms(50);
		stop();
	}
	
	
}
//function to implement picking up of block by robotic arm
void pickup(){
	
	servo_1(0);
	_delay_ms(100) ;
	servo_2(0);
	_delay_ms(100) ;
	servo_1(25) ;
	_delay_ms(100);
}

//function to implement dropping of block by robotic arm 
void drop(){
	servo_2(30);
	_delay_ms(200);
	
}

// path for traversal of each block collection.
// logic :- Path of the robot to each block is predefined using above functions wherever required.
//			whenever a specific block is to be collected, the path leading to the block is called.  
void pathto1()
{
	
	for( int i =0; i<11;i++){
		lcd_print(1,1,count,3);
		
		if(count==0){
			stop();
			break;
		}
		else if(count==1){
			rotation_using_blackline_right();
			_delay_ms(50);
			forward_black();
			++count;
			_delay_ms(50);
			
		}
		else if(count==2){
			forward_from_node();
			_delay_ms(50);
			++count;
		
		}
		else if(count==3){
			
			rotation_using_blackline_newleft();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
		
		}
		
		else if(count ==4)
		{
			forward_from_node();
			_delay_ms(50);
			++count;
			
		}
		else if(count==5){
			data = info_table[0][0]; 			  
			forward_mm(60);
			_delay_ms(50);
			right_degrees(45);
			_delay_ms(50);
			turn_right();
			_delay_ms(500);
			pickup() ;	
			velocity(120,60);
			rotation_blackline_right();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
			
		}
		else if(count == 6)
		{
			forward_from_node();
			_delay_ms(50);
			++count;
		}
		else if(count==7){
			velocity(120,60);
			rotation_using_blackline_newright();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
			
		}
		else if(count==8){
			forward_from_node();
			_delay_ms(50);
			
			++count;
			
		}
		else if(count==9){
			forward_mm(88);
			_delay_ms(50);
			rotation_using_blackline_right();
			_delay_ms(50);
			turn_right();
			_delay_ms(50);
			forward_black();
			++count;
			
		}
		else if(count==10){
			forward_from_node();
			_delay_ms(50);
			drop();
			data = info_table[0][0] + 4; 		// so as to send a particular data dependent on the block color to arduino xbee for regaining the originsl configuration 
			left_degrees(45);
			_delay_ms(50);
			rotation_blackline_left();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			forward_from_node();
			
			reset_count();									//call count reset function to reset it to 0
		}
	}
	
}

void pathto2(){
	for( int i =0; i<9;i++){
		lcd_print(1,1,count,3);
		                                    
	if(count==0){
		stop();
		break;
	}
	else if(count==1){
		rotation_using_blackline_right();
		_delay_ms(50);
		forward_black();
		++count;
		_delay_ms(50);
		
	}
	else if(count==2){
		forward_from_node();
		_delay_ms(50);
		++count;
		
	}
	else if(count==3){
		
		rotation_using_blackline_newleft();
		_delay_ms(50);
		forward_black();
		++count;
		
	}
	else if(count==4){
		data = info_table[0][1] ;  		  
		velocity(120,60);
		rotation_using_blackline_newright();
		_delay_ms(5000);
		pickup();	
		right_degrees(45);
		_delay_ms(50);
		rotation_blackline_right();
		_delay_ms(50);
		forward_black();
		_delay_ms(50);
		++count;
		
	}
	else if(count==5){
		velocity(120,60);
		rotation_using_blackline_newright();
		_delay_ms(50);
		forward_black();
		_delay_ms(50);
		++count;
		
	}
	else if(count==6){
		forward_from_node();
		_delay_ms(50);
		
		++count;
		
	}
	else if(count==7){
		rotation_using_blackline_right();
		_delay_ms(50);
		turn_right();
		_delay_ms(50);
		forward_black();
		++count;
		
	}
	else if(count==8){
		forward_from_node();
		_delay_ms(50);
		drop();
		data = info_table[0][1] + 4 ; 
		left_degrees(45);
		_delay_ms(50);
		turn_left();
		_delay_ms(50);
		forward_black();
		_delay_ms(50);
		forward_from_node();
		
		reset_count();									//call count reset function to reset it to 0
	}
}
}

void pathto3()
{
	for( int i =0; i<9;i++){
		lcd_print(1,1,count,3);
		
		if(count==0){
			stop();
			break;
		}
		else if(count==1){
			rotation_using_blackline_left();
			_delay_ms(50);
			forward_black();
			++count;
			_delay_ms(50);
			
		}
		else if(count==2){
			forward_from_node();
			_delay_ms(50);
			++count;
			
		}
		else if(count==3){
			
			rotation_using_blackline_newright();
			_delay_ms(50);
			forward_black();
			++count;
			
		}
		else if(count==4){
			data = info_table[0][2]; 			   
			velocity(60,120);
			rotation_using_blackline_newleft();
			_delay_ms(5000);
			pickup();	
			left_degrees(45);
			_delay_ms(50);
			rotation_blackline_left();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
			
		}
		else if(count==5){
			velocity(60,120);
			rotation_using_blackline_newright();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
			
		}
		else if(count==6){
			forward_from_node();
			_delay_ms(50);
			
			++count;
			
		}
		else if(count==7){
			rotation_using_blackline_right();
			_delay_ms(50);
			turn_right();
			forward_black();
			++count;
			
		}
		else if(count==8){
			forward_from_node();
			_delay_ms(50);
			drop();
			data = info_table[0][2] + 4 ; 
			left_degrees(45);
			_delay_ms(50);
			rotation_blackline_left();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			forward_from_node();
			
			
			reset_count();									//call count reset function to reset it to 0
		}
	}
}

void pathto4()
{
	for( int i =0; i<11;i++){
		lcd_print(1,1,count,3);
		
		if(count==0){
			stop();
			break;
		}
		else if(count==1){
			rotation_using_blackline_left();
			_delay_ms(50);
			forward_black();
			++count;
			_delay_ms(50);
			
		}
		else if(count==2){
			forward_from_node();
			_delay_ms(50);
			++count;
			
		}
		else if(count==3){
			
			rotation_using_blackline_newright();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
			
		}
		else if (count ==4)
		{
			forward_from_node();
			_delay_ms(50);
			++count;
		}
		else if(count==5){
			data = info_table[0][3] ;  			 
			velocity(60,120);
			rotation_using_blackline_newleft();
			_delay_ms(5000);
			pickup();	
			left_degrees(45);
			_delay_ms(50);
			rotation_blackline_left();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
			
		}
		else if(count ==6)
		{
			forward_from_node();
			_delay_ms(50);
		}
		else if(count==7){
			velocity(60,120);
			rotation_using_blackline_newright();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
			
		}
		else if(count==8){
			forward_from_node();
			_delay_ms(50);
			
			++count;
			
		}
		else if(count==9){
			rotation_using_blackline_right();
			_delay_ms(50);
			turn_right();
			_delay_ms(50);
			forward_black();
			++count;
			
		}
		else if(count==10){
			forward_from_node();
			_delay_ms(50);
			drop();
			data = info_table[0][3] + 4 ; 
			left_degrees(45);
			_delay_ms(50);
			rotation_blackline_left();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			forward_from_node();
			
			
			reset_count();									//call count reset function to reset it to 0
		}
	}
	
}

void pathto5(){
	for( int i =0; i<12;i++){
		lcd_print(1,1,count,3);
		if(count==0){
			stop();
			break;
		}
		else if(count==1){			
			forward_from_node();
			_delay_ms(50);
			++count;
			
		}
		else if(count==2){
			forward_from_node();
			_delay_ms(50);
			++count;
			
		}
		else if(count==3){
			
			rotation_using_blackline_newleft();
			_delay_ms(50);
			forward_black();
			++count;
		}
		else if(count==4){
			forward_from_node();
			_delay_ms(50);
			++count;
			
		}
		else if(count==5){
			
			data = info_table[0][4];   			   
			velocity(120,60);
			rotation_using_blackline_newright();
			_delay_ms(5000);
				pickup();	
			right_degrees(45);
			_delay_ms(50);
			rotation_blackline_right();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
		}
		else if(count==6){
			forward_from_node();
			_delay_ms(50);
			++count;
		}
		else if(count==7){
			velocity(120,60);
			rotation_using_blackline_newright();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
			
		}
		else if(count==8){
			forward_from_node();
			_delay_ms(50);
			++count;
											//call count reset function to reset it to 0
		}
		else if(count==9)
		{
			forward_from_node();
			_delay_ms(50);
			++count;
		}
		else if(count == 10)
		{
			forward_from_node();
			_delay_ms(50);
			++count;
			
		}
		else if(count == 11)
		{	drop();
			data = info_table[0][4] + 4 ; 
			left_degrees(45);
			_delay_ms(50);
			rotation_blackline_left();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			forward_from_node();
			_delay_ms(50);
			forward_from_node();
			_delay_ms(50);
			reset_count();
		}
	}
}

void pathto6()
{
	for( int i =0; i<10;i++){
		lcd_print(1,1,count,3);
		
		if(count==0){
			stop();
			break;
		}
		else if(count==1){
			forward_from_node();
			_delay_ms(50);
			++count;
			
		}
		else if(count==2){
			forward_from_node();
			_delay_ms(50);
			++count;
			
		}
		else if(count==3){
			
			rotation_using_blackline_newleft();
			_delay_ms(50);
			forward_black();
			++count;
		}
	
		else if(count==4){
			
			data = info_table[0][5] ;  			   
			velocity(120,60);
			rotation_using_blackline_newright();
			_delay_ms(5000);
			pickup();	
			right_degrees(45);
			_delay_ms(50);
			rotation_blackline_right();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
		}
		else if(count==5){
			velocity(120,60);
			rotation_using_blackline_newright();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
			
		}
		else if(count==6){
			forward_from_node();
			_delay_ms(50);
			++count;
			
		}
		else if(count==7)
		{
			forward_from_node();
			_delay_ms(50);
			++count;
		}
		else if(count == 8)
		{
			forward_from_node();
			_delay_ms(50);
			++count;
			
		}
		else if(count == 9)
		{	drop();
			data = info_table[0][5] + 4 ; 
			left_degrees(45);
			_delay_ms(50);
			rotation_blackline_left();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			forward_from_node();
			_delay_ms(50);
			forward_from_node();
			_delay_ms(50);
			reset_count();
		}
	}
	
}

void pathto7()
{
	
	for( int i =0; i<10;i++){
		lcd_print(1,1,count,3);
		
		if(count==0){
			stop();
			break;
		}
		else if(count==1){
			forward_from_node();
			_delay_ms(50);
			++count;
			
		}
		else if(count==2){
			forward_from_node();
			_delay_ms(50);
			++count;
			
		}
		else if(count==3){
			
			rotation_using_blackline_newright();
			_delay_ms(50);
			forward_black();
			++count;
		}
		
		else if(count==4){
			
			data = info_table[0][6]; 
			velocity(60,120);
			rotation_using_blackline_newleft();
			_delay_ms(5000);
			pickup();
			
			left_degrees(45);
			_delay_ms(50);
			rotation_blackline_left();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
		}
		else if(count==5){
			velocity(60,120);
			rotation_using_blackline_newleft();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
			
		}
		else if(count==6){
			forward_from_node();
			_delay_ms(50);
			++count;
			
		}
		else if(count==7)
		{
		forward_from_node();
		_delay_ms(50);
		++count;
	}
	else if(count == 8)
	
		{
			forward_from_node();
			_delay_ms(50);
			
			++count;
		}
		
		else if(count == 9)
		{   drop();
			data = info_table[0][6] + 4 ; 
			left_degrees(45);
			_delay_ms(50);
			
			rotation_blackline_left();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			forward_from_node();
			_delay_ms(50);
			forward_from_node();
			_delay_ms(50);
			reset_count();
		}
	}
}

void pathto8()
{
	
	for( int i =0; i<12;i++){
		lcd_print(1,1,count,3);
		
		if(count==0){
			stop();
			break;
		}
		else if(count==1){
			forward_from_node();
			_delay_ms(50);
			++count;
			
		}
		else if(count==2){
			forward_from_node();
			_delay_ms(50);
			++count;
			
		}
		else if(count==3){
			
			rotation_using_blackline_newright();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
		}
		else if(count == 4)
		{
			forward_from_node();
			_delay_ms(50);
			++count;
		}
		
		else if(count==5){
			
			data = info_table[0][7] ;  			   
			velocity(60,120);
			rotation_using_blackline_newleft();
			_delay_ms(5000);
			pickup();
			left_degrees(45);
			_delay_ms(50);
			rotation_blackline_left();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
		}
		else if(count == 6)
		{
			forward_from_node();
			_delay_ms(50);
			++count;
		}
		else if(count==7){
			velocity(60,120);
			rotation_using_blackline_newleft();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
			
		}
		else if(count==8){
			forward_from_node();
			_delay_ms(50);
			++count;
			
		}
		else if(count==9){
		forward_from_node();
		_delay_ms(50);
		++count;
	}
	else if(count == 10)
	{
		
			forward_from_node();
			_delay_ms(50);
			
			++count;
		}
		
		else if(count == 11)
		{	drop();
			data = info_table[0][7] + 4; 
			left_degrees(45);
			_delay_ms(50);
			rotation_blackline_left();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			forward_from_node();
			_delay_ms(50);
			forward_from_node();
			_delay_ms(50);
			reset_count();
		}
	}
}

void pathto9()
{
	for( int i =0; i<12;i++){
		lcd_print(1,1,count,3);
		//block at coordinate 2
		if(count==0){
			stop();
			break;
		}
		else if(count==1){
			rotation_using_blackline_right();
			_delay_ms(50);
			forward_black();
			++count;
			_delay_ms(50);
			
		}
		else if(count==2){
			forward_from_node();
			_delay_ms(50);
			++count;
			
		}
		else if(count==3){
			
			rotation_using_blackline_newleft();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
		
		}
		else if (count ==4)
		{
			forward_from_node();
			_delay_ms(50);
			++count;
		}
		else if(count==5){
			data = info_table[0][8];  			 
			velocity(120,60);
			rotation_using_blackline_newright();
			_delay_ms(5000);
			pickup();
			right_degrees(45);
			_delay_ms(50);
			rotation_blackline_right();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
			
		}
		else if(count ==6)
		{
			forward_from_node();
			_delay_ms(50);
			++count;
		}
		else if(count==7){
			velocity(60,120);
			rotation_using_blackline_newright();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
			
		}
		else if(count==8){
			forward_from_node();
			_delay_ms(50);
			
			++count;
		
		}
		else if(count==9){
			rotation_using_blackline_newleft();
			_delay_ms(50);
			turn_left();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
			
		}
		else if(count ==10)
		{
			forward_from_node();
			_delay_ms(50);
			++count;
		}
		else if(count==11){
			forward_from_node();
			_delay_ms(50);
			drop();
			data = info_table[0][8] + 4 ; 
			left_degrees(45);
			_delay_ms(50);
			rotation_blackline_left();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			forward_from_node();
			
		
			reset_count();									//call count reset function to reset it to 0
		}
	}
}

void pathto10()
{
	
	for( int i =0; i<10;i++){
		lcd_print(1,1,count,3);
		
		if(count==0){
			stop();
			break;
		}
		else if(count==1){
			rotation_using_blackline_right();
			_delay_ms(50);
			forward_black();
			++count;
			_delay_ms(50);
			
		}
		else if(count==2){
			forward_from_node();
			_delay_ms(50);
			++count;
			
		}
		else if(count==3){
			
			rotation_using_blackline_newleft();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
			
		}
		else if(count==4){
			data = info_table[0][9] ;  			   
			velocity(120,60);
			rotation_using_blackline_newright();
			_delay_ms(5000);
			pickup();
			right_degrees(45);
			_delay_ms(50);
			rotation_blackline_right();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
		
		}
		else if(count==5){
			velocity(60,120);
			rotation_using_blackline_newright();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
			
		}
		else if(count==6){
			forward_from_node();
			_delay_ms(50);
			
			++count;
			
		}
		else if(count==7){
			rotation_using_blackline_newleft();
			_delay_ms(50);
			turn_left();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
			
		}
		else if(count ==8)
		{
			forward_from_node();
			_delay_ms(50);
			++count;
		}
		else if(count==9){
			forward_from_node();
			_delay_ms(50);
			drop();
			data = info_table[0][9]; 
			left_degrees(45);
			_delay_ms(50);
			rotation_blackline_left();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			forward_from_node();
			
			reset_count();									//call count reset function to reset it to 0
		}
	}
	
}

void pathto11()
{
	
	for( int i =0; i<10;i++){
		lcd_print(1,1,count,3);
		
		if(count==0){
			stop();
			break;
		}
		else if(count==1){
			rotation_using_blackline_right();
			_delay_ms(50);
			forward_black();
			++count;
			_delay_ms(50);
			
		}
		else if(count==2){
			forward_from_node();
			_delay_ms(50);
			++count;
			
		}
		else if(count==3){
			
			rotation_using_blackline_newright();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
		
		}
		else if(count==4){
			data = info_table[0][10] ;  			  
			velocity(120,60);
			rotation_using_blackline_newleft();
			_delay_ms(5000);
			pickup();
			left_degrees(45);
			_delay_ms(50);
			rotation_blackline_left();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
			
		}
		else if(count==5){
			velocity(60,120);
			rotation_using_blackline_newright();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
			
		}
		else if(count==6){
			forward_from_node();
			_delay_ms(50);
			
			++count;
		
		}
		else if(count==7){
			rotation_using_blackline_newleft();
			_delay_ms(50);
			turn_left();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
			
		}
		else if(count ==8)
		{
			forward_from_node();
			_delay_ms(50);
			++count;
		}
		else if(count==9){
			forward_from_node();
			_delay_ms(50);
			drop();
			data = info_table[0][10]+ 4; 
			left_degrees(45);
			_delay_ms(50);
			rotation_blackline_left();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			forward_from_node();
		
			reset_count();									//call count reset function to reset it to 0
		}
	}
	
}

void pathto12()
{
	for( int i =0; i<12;i++){
		lcd_print(1,1,count,3);
		
		if(count==0){
			stop();
			break;
		}
		else if(count==1){
			rotation_using_blackline_right();
			_delay_ms(50);
			forward_black();
			++count;
			_delay_ms(50);
			
		}
		else if(count==2){
			forward_from_node();
			_delay_ms(50);
			++count;
			
		}
		else if(count==3){
			
			rotation_using_blackline_newright();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
			
		}
		else if (count ==4)
		{
			forward_from_node();
			_delay_ms(50);
			++count;
		}
		else if(count==5){
			data = info_table[0][11];  			  
			velocity(120,60);
			rotation_using_blackline_newleft();
			_delay_ms(5000);
			pickup();
			left_degrees(45);
			_delay_ms(50);
			rotation_blackline_left();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
			
		}
		else if(count ==6)
		{
			forward_from_node();
			_delay_ms(50);
			++count;
		}
		else if(count==7){
			velocity(60,120);
			rotation_using_blackline_newright();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
			
		}
		else if(count==8){
			forward_from_node();
			_delay_ms(50);
			
			++count;
			
		}
		else if(count==9){
			rotation_using_blackline_newleft();
			_delay_ms(50);
			turn_left();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			++count;
		
		}
		else if(count ==10)
		{
			forward_from_node();
			_delay_ms(50);
			++count;
		}
		else if(count==11){
			forward_from_node();
			_delay_ms(50);
			drop();
			data = info_table[0][11] + 4;
			left_degrees(45);
			_delay_ms(50);
			rotation_blackline_left();
			_delay_ms(50);
			forward_black();
			_delay_ms(50);
			forward_from_node();
			
			reset_count();									//call count reset function to reset it to 0
		}
	}
}

int main(void)
{
	//initializing the required functions or ports etc //
        
		init_devices();
		lcd_set_4bit();
		lcd_init();
		servo_1(25);
		servo_2(30);
		
		
            Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
            if(Left_white_line>0x20 && Center_white_line>0x20 && Right_white_line > 0x20 )
           {
			   int i,j;
			   for(i =0;i<12;++i)
			   {
			    if(i==0 && info_table[i][0])				
				{
				count = 1;
				pathto1();
				}
				else if(i ==1 && info_table[i][0])
				{
					count = 1;
					pathto2();
				}					
			   else if (i ==2 && info_table[i][0])
			   {
				   count = 1;
				   pathto3();
			   }
			   else if(i ==3 && info_table[i][0])
			   {
				   count = 1;
				   pathto4();
			   }
			   else if(i ==4 && info_table[i][0])
			   {
				   count = 1;
				   pathto5();
			   }
			   else if(i ==5 && info_table[i][0])
			   {
				   count = 1;
				   pathto6();
			   }
			   else if(i ==6 && info_table[i][0])
			   {
				    count = 1;
				    pathto7();
			   }
			   else if(i ==7 && info_table[i][0])
			   {
				   count = 1;
				   pathto8();
			   }
			   else if(i ==8 && info_table[i][0])
			   {
				    count = 1;
				    pathto9();
			   }
			   else if(i ==9 && info_table[i][0])
			   {
				   count = 1;
				   pathto10();
			   }
			   else if(i ==10 && info_table[i][0])
			   {
				   count = 1;
				   pathto11();
			   }
			   else if(i ==11 && info_table[i][0])
			   {
				   count = 1;
				   pathto12();
			   }
			   }
			   
			   
			   
			   }
				buzzer_on();
            }
			
      
	
	
	
	