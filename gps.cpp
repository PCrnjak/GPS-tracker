#include "mbed.h"
#include "stdio.h"
#include "math.h"
#include <string>
/*****************************************************************************
* Tells us after how many commas is data we want
******************************************************************************/
#define TIME 1
#define LATITUDE 2
#define NS 3
#define LONGITUDE 4
#define EW 5
#define QUALITY 6
#define SATELLITES 7
#define HDOP 8
#define ALTITUDE 9

AnalogIn Bat_pin(PA_4);
DigitalOut LED(PA_7);
DigitalOut LED_BUILTIN(PC_13);
//DigitalIn sw1(PB_1);
DigitalIn sw2(PB_0); // switch
InterruptIn sw1(PB_1); // button
/*         TX     RX         */
Serial pc(PA_9, PA_10); // Debug serial
Serial GPS(PB_10, PB_11); // GPS serial
Serial GSM(PA_2,PA_3);   // GSM serial

char buffer[100]; // We save our data here
char buffer2[50]; // Array that that is later transformed to int, floats...

int elements = 0;
void get_data(void);
char check_Quality(void);
void get_Time(void);
void get_Latitude(void);
void get_Longitude(void);
void get_Satellites(void);
void get_HDOP(void);
void get_Altitude(void);
float get_battery_voltage(void);
void SIM800L_diagnostic(void);
bool Device_diagnostic(void);
void send_data(void);

/* We save our GPS data here */
double Latitude = 0.0;
char N_S_indicator = ' ';
double Longitude = 0.0;
char E_W_indicator = ' ';
int Sat_used = 0;
double Hdop = 0.0;
double Altitude = 0.0;
float sim800L_battery = 0.0;
float batV = 0.0;
uint8_t sim800L_reg = 0;
bool device = 0;


int h = 0;
int m = 0;
int s = 0;

Timer Send_timer_1; // When we send out data (automatic mode)
Timer Send_timer_2; // When we can send data  (manual mode)
Timer Diagnostic_timer; // When we run device diagnostic
Timer Blink_timer; // used to blink the LED
Timer debounce; // uesd to debounce switch
bool cond2 = 0; // return of Device_diagnostic();
bool cond1 = 0; // 1 if it is time to send data
volatile bool button_cond = 0; // if it is not volatile it will not work
float blink_time = 0.2;


/*****************************************************************************
* If the button is pressed set button_cond to 1
* After that we cant send any data in next 5 seconds
******************************************************************************/
void isr()
{
    if (debounce.read_ms() > 5000) {
        button_cond = 1;
        debounce.reset();
    }
}


int main()
{
    pc.baud (9600);
    GSM.baud (9600);
    GPS.baud (9600);

    sw2.mode(PullUp);
    sw1.mode(PullUp);
    sw1.rise(&isr);

    Send_timer_1.start();
    Diagnostic_timer.start();
    Blink_timer.start();
    debounce.start();

    wait_ms(1000);
    pc.printf("\r\n");
    pc.printf("Device is setting up\r\n");
    wait_ms(13000);
    get_data();
    batV = get_battery_voltage();
    wait_ms(1000);


    while(1) {

     // if battery is less then 3.7V disable everything , LED is permanently ON
        if(batV >=3.8) {

            LED_BUILTIN = sw2;

            if (Diagnostic_timer.read() >= 10) {
                cond2 = Device_diagnostic(); // returns 1 if good data 0 if bad data
                if(cond2 == 0) {
                    blink_time = 0.65; // if data is bad we blink every 0.65 seconds
                } else {
                    blink_time = 0.15; // if data is good we blink every 0.2 seconds
                }
                Diagnostic_timer.reset();
            }

            if(Blink_timer.read() >= blink_time) {
                LED = !LED;
                Blink_timer.reset();
            }

            /*****************************************************************************
            * If switch is in top position we are in automatic mode
            * if switch is in bottom positon we are in manual mode
            * in manual mode we can send data by pressing button
            * data will be sent only if Device_diagnostic() returns good data
            * if not it will send nothing . We can send data every 10 seconds only
            ******************************************************************************/
            if(sw2 == 0) {

                /* sends data every 60 seconds */
                if (Send_timer_1.read() >= 60) {
                    cond1 = 1;
                    Send_timer_1.reset();
                }

                /* Send data if desired time pased and device is working */
                if(cond1 == 1 and cond2 == 1) {
                    get_Latitude();
                    get_Longitude();
                    get_Satellites();
                    get_HDOP();
                    get_Altitude();
                    send_data();
                    cond1 = 0;


                }

            } else {


     /*sends data only if desired time passed,device is working,and we pressed the button */
                if(button_cond == 1 and cond2 == 1) {
                    get_Latitude();
                    get_Longitude();
                    get_Satellites();
                    get_HDOP();
                    get_Altitude();
                    send_data();
                    button_cond = 0;


                }


            }
        } else

            /* Low battery ,Nothing is working */
            LED =1;
    }

}





/*****************************************************************************
* Gets time from GPS module
******************************************************************************/
void get_Time(void)
{

    int cnt = 6;
    int time_ = 0;
    int slash = 6; // After what comma we start reading data
    bool cond = 0;

    while(cond == 0) {
        if(buffer[cnt] == ',') {
            cond = 1;
        } else {
            buffer2[cnt-slash] = buffer[cnt];
            cnt = cnt + 1;
        }
        if(cond == 1) {
            //Must add '\0' to the end of array if we want atoi to work
            buffer2[cnt-slash] = '\0' ; 
            time_ = atoi(buffer2); // transform char array to integer
        }

    }
    /* Universal Time Coordinated (UTC) in hhmmss.ss. */
    h = time_ / 10000 + 1; // Time is shifted by -1 h , so need to +1 h
    s = time_ % 100;
    m = (time_ / 100) % 100;

}

/*****************************************************************************
* Latitude mesures how far north or south a point is from Equator
* Equator is imaginary line half way between north and south pole
******************************************************************************/
void get_Latitude(void)
{

    int cnt = 6;
    int cnt2 = 0;
    int slash = 0;
    bool cond = 0;

    while(cond == 0) {
        if(buffer[cnt] == ',') {
            slash = slash + 1;
        }
        cnt = cnt + 1;
        // LATITUDE is usually 2 commas but we ignore first 6 chars so it is only 1 
        if(slash == LATITUDE - 1) { 
            while(buffer[cnt] != ',') {// read until we find comma
                buffer2[cnt2] = buffer[cnt] ;
                //  pc.printf(" char is %d is  %c \r\n",cnt2,buffer2[cnt2]);
                cnt = cnt + 1;
                cnt2 = cnt2 +1;
            }
            cond = 1;
            buffer2[cnt2 ] = '\0'; //// add '\0' for atof to work
            Latitude = atof(buffer2);  
            pc.printf(" Latitude float is  %f \r\n",Latitude);  
            N_S_indicator = buffer[cnt + 1];   //Take  N_S indicator 
            pc.printf(" N_S is  %c \r\n",N_S_indicator); 
            int deg = Latitude / 100;
            double min = (fmod(Latitude,100))/60;
            Latitude = deg + min ; 
            pc.printf(" Latitude float is  %f \r\n",Latitude); 

        }

    }
}



/*****************************************************************************
* Longitude mesures how far west or east a point is from prime meridian ( Greenwich)
******************************************************************************/
void get_Longitude(void)
{

    int cnt = 6;
    int cnt2 = 0;
    int slash = 0;
    bool cond = 0;

    while(cond == 0) {
        if(buffer[cnt] == ',') {
            slash = slash + 1;
        }
        cnt = cnt + 1;
        // LONGITUDE is usually 4 commas but we ignore first 6 chars so it is only 3 
        if(slash == LONGITUDE  - 1) { 
            while(buffer[cnt] != ',') { // read until we find comma
                buffer2[cnt2] = buffer[cnt] ;
                //  pc.printf(" char is %d is  %c \r\n",cnt2,buffer2[cnt2]);
                cnt = cnt + 1;
                cnt2 = cnt2 +1;
            }
            cond = 1;
            buffer2[cnt2 ] = '\0'; //// add '\0' for atof to work
            Longitude = atof(buffer2);  
            pc.printf(" Longitude float is  %f \r\n",Longitude);  
            E_W_indicator = buffer[cnt + 1];   
            pc.printf(" E_W is  %c \r\n",E_W_indicator); 
            int deg = Longitude / 100;
            double min = (fmod(Longitude,100))/60;
            Longitude = deg + min ;
            pc.printf(" Longitude float is  %f \r\n",Longitude); 

        }

    }
}




void get_Altitude(void)
{

    int cnt = 6;
    int cnt2 = 0;
    int slash = 0;
    bool cond = 0;

    while(cond == 0) {
        if(buffer[cnt] == ',') {
            slash = slash + 1;
        }
        cnt = cnt + 1;
         // ALTITUDE is usually 9 commas but we ignore first 6 chars so it is only 8
        if(slash == ALTITUDE  - 1) { 
            while(buffer[cnt] != ',') {  // read until we find comma
                buffer2[cnt2] = buffer[cnt] ;
                // pc.printf(" char is %d is  %c \r\n",cnt2,buffer2[cnt2]);
                cnt = cnt + 1;
                cnt2 = cnt2 +1;
            }
            cond = 1;
            buffer2[cnt2 ] = '\0'; //// add '\0' for atof to work
            Altitude = atof(buffer2);  
            pc.printf(" Altitude float is in M %f \r\n",Altitude); 

        }

    }
}







void get_Satellites(void)
{

    int cnt = 6;
    int cnt2 = 0;
    int slash = 0;
    bool cond = 0;

    while(cond == 0) {
        if(buffer[cnt] == ',') {
            slash = slash + 1;
        }
        cnt = cnt + 1;
        // SATELLITES is usually 7 commas but we ignore first 6 chars so it is only 6
        if(slash == SATELLITES  - 1) { 
            while(buffer[cnt] != ',') {  // read until we find comma
                buffer2[cnt2] = buffer[cnt] ;
                //pc.printf(" char is %d is  %c \r\n",cnt2,buffer2[cnt2]);
                cnt = cnt + 1;
                cnt2 = cnt2 +1;
            }
            cond = 1;
            buffer2[cnt2 ] = '\0'; //// add '\0' for atof to work
            Sat_used = atoi(buffer2);  
            


        }

    }
}


/*****************************************************************************
* HDOP is measure of confidence in the solution
******************************************************************************/
void get_HDOP(void)
{

    int cnt = 6;
    int cnt2 = 0;
    int slash = 0;
    bool cond = 0;

    while(cond == 0) {
        if(buffer[cnt] == ',') {
            slash = slash + 1;
        }
        cnt = cnt + 1;
        // HDOP is usually 8 commas but we ignore first 6 chars so it is only 7
        if(slash == HDOP  - 1) { 
            while(buffer[cnt] != ',') {  // read until we find comma
                buffer2[cnt2] = buffer[cnt] ;
                //pc.printf(" char is %d is  %c \r\n",cnt2,buffer2[cnt2]);
                cnt = cnt + 1;
                cnt2 = cnt2 +1;
            }
            cond = 1;
            buffer2[cnt2 ] = '\0'; //// add '\0' for atof to work
            Hdop = atof(buffer2);  
            pc.printf(" HDOP je  %f (manji broj je bolje) \r\n",Hdop);  


        }

    }
}





/*****************************************************************************
*what kind of fix the receiver has obtained.
******************************************************************************/
char check_Quality(void)
{
    int cnt = 0;
    int slash = 0;
    bool cond = 0;
    while(cond == 0) {
        if(buffer[cnt] == ',') {
            slash = slash + 1;
        }
        cnt = cnt + 1;
        if(slash == QUALITY and cond == 0) {
            // pc.printf("char is %c\r\n",buffer[cnt]);
            switch (buffer[cnt]) {

                case '0':   // 0 — No fix

                    pc.printf("No fix\r\n");
                    cond = 1;
                    break;

                case '1':   // 1 — Standard GPS (2D/3D) fix

                    pc.printf("Standard GPS (2D/3D) fix\r\n");
                    cond = 1;
                    break;

                case '2':  // 2 — Differential GPS (DGPS) fix

                    pc.printf("Differential GPS (DGPS) fix\r\n");
                    cond = 1;
                    break;

                case '3':  // 3 — Precise Positioning System (PPS) fix for government use only

                    pc.printf("Precise Positioning System (PPS) fix for government use only\n");
                    cond = 1;
                    break;

                case '4': // 4 — RTK fixed solution

                    pc.printf("RTK fixed solution\n");
                    cond = 1;
                    break;

                case '5': // 5 — RTK float solution

                    pc.printf("RTK float solution\n");
                    cond = 1;
                    break;

                case '6':  // 6 — Estimated (Dead Reckoning, or DR) fix

                    pc.printf("Estimated (Dead Reckoning, or DR) fix\n");
                    cond = 1;
                    break;

            }
        }
    }
    return buffer[cnt] ;
}



/*****************************************************************************
* Read data from GSM module
* Look for one that starts with GPGGA
******************************************************************************/
void get_data(void)
{
    int num_ = 5; // we have 5 chars after $
    int flag = 0; // conditon if we found GPGGA
    bool end = 0; // main condition for reading
    while(end == 0) {
        while(GPS.getc() !='$') {  // Look for $ , every NMEA Sentences starts with that
        }

 //If next sequance of chars is GPGGA , that is data we are looking for and we can continue
        if (GPS.getc() == 'G') { 
            buffer[0] = 'G';
            if (GPS.getc() == 'P') { 
                buffer[1] = 'P';
                if (GPS.getc() == 'G') { 
                    buffer[2] = 'G';
                    if (GPS.getc() == 'G') { 
                        buffer[3] = 'G';
                        if (GPS.getc() == 'A') { 
                            buffer[4] = 'A';
                            num_ = 5;

                            flag = 0;
                            while(flag == 0) {
                                char var = GPS.getc();
                        /* If we find \r we are at the end of sequance and we stop reading */
                        /* Our data looks like this $GPGGA,data,data,data.....\r\n */
                                if(var != '\r') {
                                    buffer[num_] = var;
                                    num_ = num_ + 1;
                                    flag = 0;
                                } else {
                                    flag = 1;
                                    end = 1;
                                    elements = num_ ;
                                }
                            }

                          
                            //   }
                        }
                    }
                }
            }
        }
    }

}

/*****************************************************************************
* Get battery voltage from our ADC
******************************************************************************/
float get_battery_voltage()
{
    uint16_t V_Bat = 0;
    float V_batf = 0;
    V_Bat = Bat_pin.read_u16();
    V_Bat = V_Bat >> 4;
    V_batf = ((float)V_Bat / 4096)*4.2;
    return V_batf;

}

/*****************************************************************************
* Get battery voltage from our ADC
******************************************************************************/
void send_data()
{
    char send_command = 0x1A;
    GSM.printf("AT+CMGF=1\r");    // AT command to set module to SMS text mode
    wait_ms(100);
    GSM.printf("AT+CMGS=\"+385958405100\"\r");//AT command to set number of receiver
    wait_ms(1000);
    // data we send
    GSM.printf("Coordinates are %.5f %c %.5f %c",Latitude,N_S_indicator,Longitude,E_W_indicator);
    GSM.printf("\r");
    GSM.printf(&send_command);
    wait_ms(100);
}



/*****************************************************************************
* Get data from SIM800L module
******************************************************************************/
void SIM800L_diagnostic()
{

    char dat_1[30],dat_2[30];
    char buff[20];
    /*****************************************************************************
    * ATE0 -> Command for the TURN OFF the repated terms(ECHO).
    * Response will be saved in x and y variables
    ******************************************************************************/
    GSM.printf("ATE0\r\n");             //Command for the TURN OFF the repated terms(ECHO).
    GSM.scanf("%s",dat_1);              //Take response is string x given by the "sim800".
    pc.printf("AT is %s\r\n",dat_1);


    /*****************************************************************************
    * Check battery level. First number is % second is voltage
    ******************************************************************************/
    GSM.printf("AT+CBC\r\n");  // send command to SIM800L module
    GSM.scanf("%s %s",dat_1,dat_2);
    //pc.printf("Response is  %s\r\n",y);
    // We know at what position voltage reading is so we can just extract it
    for(int i = 5 ; i<=9 ; i++) {
        buff[i-5] = dat_2[i];
        int var = atoi(buff);
        sim800L_battery = (float)var / 1000; // transform voltage to float
    }
    // pc.printf("Battery voltage %.3f V \r\n",sim800L_battery);



    /*****************************************************************************
    * Check signal strenght
    * Signal is first number
    * 0 is no signal 31 is max signal
    ******************************************************************************/
    GSM.printf("AT+CSQ  \r\n"); // send command to SIM800L module
    GSM.scanf("%s %s",dat_1,dat_2);
    pc.printf("Signal is  %s\r\n",dat_2);



    /*****************************************************************************
    *Get SIM card number
    ******************************************************************************/
    GSM.printf(" AT+CCID  \r\n");  // send command to SIM800L module
    GSM.scanf("%s %s",dat_1,dat_2);
    pc.printf("SIM card number is %s \r\n",dat_1);


    /*****************************************************************************
    * Check that you’re registered on the network.
    * The second # should be 1 or 5. 1 indicates you are
    * registered to home network and 5 indicates roaming network.
    * Other than these two numbers indicate you are not registered to any network.
    ******************************************************************************/
    GSM.printf(" AT+CREG? \r\n"); // send command to SIM800L module
    GSM.scanf("%s %s",dat_1,dat_2);
    buff[0] =  dat_2[2];
    buff[1] = '\0';
    sim800L_reg = atoi(buff);
    if(sim800L_reg == 1) {
        pc.printf("network is %d , home network \r\n",sim800L_reg);
    } else if(sim800L_reg == 5) {
        pc.printf("network is %d , roaming  network \r\n",sim800L_reg);
    } else {
        pc.printf("No network \r\n");

    }

    /*****************************************************************************
    * Check model of SIM module.
    ******************************************************************************/
    GSM.printf("ATI  \r\n");  // send command to SIM800L module
    GSM.scanf("%s %s",dat_1,dat_2);
    pc.printf("Model of SIM module is %s %s\r\n",dat_1,dat_2);


}



/*****************************************************************************
* Useful data that is returned to pc using serial port
* Device_diagnostic() function returns 0 if device is not working  and
* 1 if device is working . Usually problems can be no GPS satellite fix or no
* GSM fix
******************************************************************************/
bool Device_diagnostic()
{

    pc.printf("\r\n\n\n\n");
    pc.printf("*********************************\r\n");
    bool GSM_GOOD = 0;
    bool GPS_GOOD = 0;

    /* Check if GSM module is working */
    SIM800L_diagnostic();
    if(sim800L_reg == 1 ||  sim800L_reg == 5) {
        GSM_GOOD = 1;
    } else {
        GSM_GOOD = 0;
    }

    /* Check if GPS module is working */
    get_data();
    char test = check_Quality();
    if( test == '0' ) {
        pc.printf("Invalid GPS data \r\n");
        GPS_GOOD = 0;
    } else {
        pc.printf("Good GPS data \r\n");
        GPS_GOOD = 1;
    }

    get_Time();
    pc.printf("Time is %d h %d m %d s \r\n",h,m,s);
    get_Satellites();
    pc.printf("Number of satellites using is: %d \r\n",Sat_used);
    float bat_v = get_battery_voltage();
    pc.printf("Voltage from SIM800L is %.3f and voltage from ADC is %.3f \r\n",sim800L_battery,bat_v);

    /*If both GPS and GSM modules are working return 1 else send 0 */
    if(GSM_GOOD == 1 && GPS_GOOD == 1) {
        pc.printf("Device is working ");
        pc.printf("*********************************\r\n");
        return 1;
    } else {
        pc.printf("Device is not working\r\n");
        pc.printf("*********************************\r\n");
        return 0;
    }

}

