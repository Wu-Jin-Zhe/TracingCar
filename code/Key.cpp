#include "Key.hpp"


zf_driver_gpio key_1(KEY_1_PATH, O_RDWR);
zf_driver_gpio key_2(KEY_2_PATH, O_RDWR);
zf_driver_gpio key_3(KEY_3_PATH, O_RDWR);
zf_driver_gpio key_4(KEY_4_PATH, O_RDWR);

#define PRESSED                 1
#define UNPRESSED               0

#define KEY_COUNT               4

#define KEY_TIME_DOUBLE			200
#define KEY_TIME_LONG			1000
#define KEY_TIME_REPEAT			100

int Key_Flag[KEY_COUNT];

int Key_GetState(int n )
{
		 if(n == KEY0 )
		 {
			if(key_1.get_level() == 0)
			{ 
				return PRESSED;
			}
		 }
		 
		 else if(n == KEY1 )
		 {
			if(key_2.get_level() == 0)
			{ 
				return PRESSED;
			}
		 }
		 
		 else if(n == KEY2 )
		 {
			if(key_3.get_level() == 0)
			{ 
				return PRESSED;
			}
		 }
		
		 else if(n == KEY3 )
		 {
			if(key_4.get_level() == 0)
			{ 
				return PRESSED;
			}
		 }
		 return UNPRESSED ;
}

void Key_Tick(void )
{
		 static int Count,i;
		 static int S[KEY_COUNT] = {0};
		 static int CurrState[KEY_COUNT],PrevState[KEY_COUNT];
		 static int Time[KEY_COUNT ];
		 
		 Count ++;
		 
	    for(i = 0;i<KEY_COUNT ;i++)
		{
			if(Time[i]>0)
			{
			   Time[i]--;
			}		    
		}
		 
		 if(Count >= 20)
		 {
		    Count = 0;
			  for(i = 0;i<KEY_COUNT ;i++)
			  {
			    PrevState[i] = CurrState[i];
					CurrState[i] = Key_GetState (i);
					
					if(CurrState[i] == PRESSED )
					{
					   Key_Flag[i] |= KEY_HOLD;
					}
					else
                    {
					   Key_Flag[i] &= ~KEY_HOLD;
					}
					
					if(CurrState[i] == PRESSED && PrevState[i] == UNPRESSED )
					{
					   Key_Flag[i] |= KEY_DOWN;
					}
					
					if(CurrState[i] == UNPRESSED && PrevState[i] == PRESSED )
					{
					   Key_Flag[i] |= KEY_UP;
					}
					
					if(S[i] == 0)
					{
					   if(CurrState[i] == PRESSED )
						 {
						    S[i] = 1;
							  Time[i] = KEY_TIME_LONG;
						 }
					}
					else if(S[i] == 1)
					{
					   if(Time[i] == 0)
						 {
						    Key_Flag [i] |= KEY_LONG;
							  S[i] = 4;
							  Time[i] = KEY_TIME_REPEAT;
						 }
						 else if(CurrState[i] == UNPRESSED)
						 {
						    S[i] = 2;
							  Time[i] = KEY_TIME_DOUBLE;
						 }
					}
					else if(S[i] == 2)
					{
					   if(Time[i] == 0)
						 {
						    S[i] = 0;
							  Key_Flag [i] |= KEY_SINGLE ;
						 }
						 else if(CurrState[i] == PRESSED )
						 {
						    Key_Flag [i] |= KEY_DOUBLE ;
							  S[i] = 3;
						 }
					}
					else if(S[i] == 3)
					{
					    if(CurrState[i] == UNPRESSED )
							{
							   S[i] = 0;
							}
					}
					else if(S[i] == 4)
					{
					    if(Time[i] == 0)
							{
									Key_Flag [i] |= KEY_REPEAT ;
									Time[i] = KEY_TIME_LONG;
									S[i] = 4;
							}
							else if(CurrState[i] == UNPRESSED)
							{
							    S[i] = 0;
							}
					}
			  }
		 }		 
}

int Key_Check(int n,int Flag)
{
    if(Key_Flag [n]&Flag)
		{
		   if(Flag != KEY_HOLD)
			 {
			    Key_Flag [n] &=~Flag;
			 }
			 return 1;
		}
		return 0 ;
}
