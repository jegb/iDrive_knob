
#include <Canbus.h>
#include <string.h>

#define CAN_INT_PIN     2  //in

//Debug defines
#define CAN_ACTIVE true
#define DEBUG_LOOP false

//iDrive knob defines
#define KNOB_ROTATE_RIGHT 20
#define KNOB_ROTATE_LEFT  21
#define KNOB_UP           22
#define KNOB_DOWN         23
#define KNOB_RIGHT        24
#define KNOB_LEFT         25
#define KNOB_PRESS        26
#define MENU_BUTTON       27

#define IDRIVE_KNOB 0x1B8

// define CPU frequency in Mhz here if not defined in Makefile or other includes, compiler will throw a warning, ignore
#ifndef F_CPU
#define F_CPU 16000000UL // 28636360UL, 14318180UL, 21477270 UL
#endif

//CANBUS variables
unsigned char read_buffer[19];  //2 cmd words + 1 length wd + 16 data wds = 19, truncate if > 16 data wds
//unsigned char read_buffer_temp[10][19];
unsigned char read_buffer_temp[19];
unsigned short can_cmd = 0;
byte can_count = 0;

//Timer vars
byte timer_active = false;
unsigned long start_time = 0;
unsigned long end_time = 0;
unsigned long sniff_start = 0;
unsigned long idrive_timer = 0;
unsigned long log_timer = 0;
unsigned long log_start = 0;

//iDrive menu globals
byte window_focus = 0;
short menu_command = 0;
boolean menu_changed = false;
unsigned int last_rotate = 0;
byte bt_activity = 0;

//Misc functions
//float read_std_obd2(unsigned short);
int get_update_param(int);
unsigned long start_stop_timer(unsigned short,unsigned short);
void find_can_id(unsigned short);
int check_for_shutdown();
void verify_config();
void command_memorycheck();
void canbus_to_serial(unsigned char *);

//Volatile variables (anything used by the ISR should go here)
volatile boolean fresh_rx = false;


//Interrupt Service Routine (ISR)
void Canbus_ISR();


void setup()                    // run once, when the sketch starts
{
   //Attach interrupt to CAN-bus interrupt line (DIO pin 2)
  attachInterrupt(0,Canbus_ISR,FALLING);
} // end setup

void loop(){
  
 if (CAN_ACTIVE == true)
  {
    if (Canbus.init(CANSPEED_100A,FILTER_ON))  // Initialize MCP2515 CAN controller at the specified speed
    {
      Serial.println("CAN Init OK");
    } 
    else
    {
      Serial.println("Cannot init CAN!/");
    }
  }
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Check for CAN Shield interrupts knob changes //////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Canbus_ISR()
{

  if (CAN_ACTIVE == false)
  {
    fresh_rx = false;
    return;
  }
  
  fresh_rx = true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read CANBUS message if interrupt fired ////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_canbus_message()
{
  unsigned int rotate_num = 0;
  
  if (fresh_rx == true)
  {
    //Read CAN message
    if (Canbus.message_rx((unsigned char*)read_buffer) == 0)
    {
      //NTSC_Term_Print("?");
      fresh_rx = false;
      return;
    }
   /* 
    //Route CAN message to serial (BT)
    if (active_screen == CAN_SNIFFER)
    {
      for (byte i=0;i<19;i++)
      {
        //read_buffer_temp[can_count][i] = read_buffer[i];
        read_buffer_temp[i] = read_buffer[i];
      }
      //canbus_to_serial(&read_buffer_temp[can_count][0]);
      canbus_to_serial(&read_buffer_temp[0]);
      //Serial.print(can_count);
      if (can_count < 9) 
      {
        //can_count++;
      }
    }
    */
    
    can_cmd = (read_buffer[1] << 8) + read_buffer[0];
    //sprintf(buff,"buf1 = 0x%x, buf0 = 0x%x, sum=0x%x",read_buffer[1],read_buffer[0],can_cmd);
    //NTSC_Term_Print(buff);
    //sprintf(buff,"2 0x%x/",read_buffer[2]);
    //NTSC_Term_Print(buff);
    //sprintf(buff,"3 0x%x/",read_buffer[3]);
    //NTSC_Term_Print(buff);
    //sprintf(buff,"4 0x%x/",read_buffer[4]);
    //NTSC_Term_Print(buff);
    //sprintf(buff,"5 0x%x/",read_buffer[5]);
    //NTSC_Term_Print(buff);
    //sprintf(buff,"6 0x%x/",read_buffer[6]);
    //NTSC_Term_Print(buff);
    //sprintf(buff,"7 0x%x/",read_buffer[7]);
    //NTSC_Term_Print(buff);
    //sprintf(buff,"8 0x%x/",read_buffer[8]);
    //NTSC_Term_Print(buff);
  
    //http://translate.google.com/translate?u=http%3A%2F%2Fwww.xolmatic.com%2Fxprojects%2FXE65%2FCAN.htm&sl=es&tl=en&hl=&ie=UTF-8  
    //Look for specific commands, anything else exit ISR
    switch (can_cmd)
    {
      //iDrive knob commands
      //I-drive Menu button: 1B8 6 0FC5nnnn206F (or 0FC4nnnn206F, sometimes C4 sometimes C5)
      //I-drive knob press: 1B8 6 0FC1nnnn206F
      //I-drive knob up: 1B8 6 00C0nnnn206F
      //I-drive knob down: 1B8 6 04C0nnnn206F
      //I-drive knob left: 1B8 6 06C0nnnn206F
      //I-drive knob right: 1B8 6 02C0nnnn206F
      //I-drive knob rotate right: 1B8 6 0FC0nnnn206F (nn is increasing)
      //I-drive knob rotate left: 1B8 6 0FC0nnnn206F (nn is decreasing)
      //  nnnn min = 0000
      //  nnnn max = FF|FF (where buffer 5 is lower and 6 is the higher byte)
       
      case IDRIVE_KNOB:  //Command word 0x1B8
        //Knob rotation or return to center
        if ((read_buffer[3] == 0x0F) && (read_buffer[4] == 0xC0)
            && (read_buffer[7] == 0x20) && (read_buffer[8] == 0x6F))
        {
          rotate_num = (read_buffer[6] << 8) + read_buffer[5];

          //I-drive knob rotate right: 1B8 6 0FC0nnnn206F (nn is increasing)
          //if (read_buffer[5] > last_rotate)
          if (rotate_num > last_rotate)          
          {
            menu_command = KNOB_ROTATE_RIGHT;
            if (window_focus != 0)
              menu_changed = true;
            //NTSC_Term_Print("Knob r right/");
            Serial.println("Knob r rt");
          }
          //I-drive knob rotate left: 1B8 6 0FC0nnnn206F (nn is decreasing)
          //else if (read_buffer[5] < last_rotate)
          else if (rotate_num < last_rotate)
          {
            menu_command = KNOB_ROTATE_LEFT;
            if (window_focus != 0)
              menu_changed = true;
            //NTSC_Term_Print("Knob r left/");
            Serial.println("Knob r lf");
          }
          //I-drive knob return to center since no rotation increase
          else
          {
            menu_changed = true;
            //NTSC_Term_Print("debnc");
            Serial.println("Return");
          }
          //last_rotate = read_buffer[5];
          last_rotate = rotate_num;
        }
        //I-drive knob up: 1B8 6 00C0nnnn206F
        else if ((read_buffer[3] == 0x00) && (read_buffer[4] == 0xC0)
            && (read_buffer[7] == 0x20) && (read_buffer[8] == 0x6F))
        {
          menu_command = KNOB_UP;
          //menu_changed = true;
          //NTSC_Term_Print("Knob up/");
          Serial.println("Knob up");
        }
        //I-drive knob down: 1B8 6 04C0nnnn206F
        else if ((read_buffer[3] == 0x04) && (read_buffer[4] == 0xC0)
            && (read_buffer[7] == 0x20) && (read_buffer[8] == 0x6F))
        {
          menu_command = KNOB_DOWN;
          //menu_changed = true;
          //NTSC_Term_Print("Knob down/");
          Serial.println("Knob down");
        }
        //I-drive knob right: 1B8 6 02C0nnnn206F
        else if ((read_buffer[3] == 0x02) && (read_buffer[4] == 0xC0)
            && (read_buffer[7] == 0x20) && (read_buffer[8] == 0x6F))
        {
          menu_command = KNOB_RIGHT;
          //menu_changed = true;
          //NTSC_Term_Print("Knob right/");
          //Serial.println("Knob right");
        }
        //I-drive knob left: 1B8 6 06C0nnnn206F
        else if ((read_buffer[3] == 0x06) && (read_buffer[4] == 0xC0)
            && (read_buffer[7] == 0x20) && (read_buffer[8] == 0x6F))
        {
          menu_command = KNOB_LEFT;
          //menu_changed = true;
          //NTSC_Term_Print("Knob left/");
          Serial.println("Knob left");
        }
        //I-drive knob press: 1B8 6 0FC1nnnn206F
        else if ((read_buffer[3] == 0x0F) && (read_buffer[4] == 0xC1)  //0xc0 is press up
            && (read_buffer[7] == 0x20) && (read_buffer[8] == 0x6F))
        {
          menu_command = KNOB_PRESS;
          //menu_changed = true;
          //NTSC_Term_Print("Knob press/");
          Serial.println("Knob press");
        }
        //I-drive Menu button: 1B8 6 0FC5nnnn206F (or 0FC4nnnn206F, sometimes C4 sometimes C5)
        else if ((read_buffer[3] == 0x0F) && ((read_buffer[4] == 0xC5) || (read_buffer[4] == 0xC4))
            && (read_buffer[7] == 0x20) && (read_buffer[8] == 0x6F))
        {
          menu_command = MENU_BUTTON;
          //menu_changed = true;
          //NTSC_Term_Print("Menu btn/");
          Serial.println("Menu btn");
        }
        break;
      //Other custom BMW commands, put here
      //  ....
      
      default:
        menu_command = 0;
        menu_changed = false;
        break;
    }
    
    fresh_rx = false;
  }
  
}  


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read Standard OBD2 messages ///////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*float read_std_obd2(unsigned short param)
{

  if (CAN_ACTIVE == false)
    return -999;

  if (Canbus.ecu_req(param,can_buffer) == 1)
  {
    return atof(can_buffer);
  }
  else
    return -999;

}*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read CAN bus for Specific Command ID //////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void find_can_id(unsigned short cmd_to_find)
{
  if (CAN_ACTIVE == false)
    return;

  //find_flag = true;
  //find_id = cmd_to_find;
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  //////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void canbus_to_serial(unsigned char *buff_in)
{
  //char buff_temp[7] = {0};
  char buff_out[16] = {0};

  sprintf(buff_out,"%dms, 0x",(int)(millis()-sniff_start));
  Serial.print(buff_out);
  //strcpy(buff_out,buff_temp);

  for (byte i=1;i<buff_in[2]+3;i+=2)
  {
    if (i == 3)
    {
      sprintf(buff_out,"%02x ",buff_in[2]);
      //strcat(buff_out,buff_temp);
      Serial.print(buff_out);
      i = 2;
    }
    else if (i > 18)
    {
      //truncate message if greater than 16 data words (19 total words)
      //Serial.println("");
      break;
    }
    else
    {
      sprintf(buff_out,"%02x %02x ",buff_in[i],buff_in[i-1]);
      Serial.print(buff_out);
      //strcat(buff_out,buff_temp);
    }
    
    //sprintf(buff_out,"%x ",buff_in[i]);
    
    if (can_count > 0)
    {
      can_count = 0;
      break;
    }
  }
  
  Serial.println("");
  
/*  if (can_count >= 1)
  {
    --can_count;
    canbus_to_serial(&read_buffer_temp[can_count][0]);
  }
*/  
}

