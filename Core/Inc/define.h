/* The basic operations perfomed on two numbers a and b of fixed
 point q format returning the answer in q format */
#define FADD(a,b) ((a)+(b))
#define FSUB(a,b) ((a)-(b))
#define FMUL(a,b,q) (((a)*(b))>>(q))
#define FDIV(a,b,q) (((a)<<(q))/(b))
/* The basic operations where a is of fixed point q format and b is
 an integer */
#define FADDI(a,b,q) ((a)+((b)<<(q)))
#define FSUBI(a,b,q) ((a)-((b)<<(q)))
#define FMULI(a,b) ((a)*(b))
#define FDIVI(a,b) ((a)/(b))
/* convert a from q1 format to q2 format */
#define FCONV(a, q1, q2) (((q2)>(q1)) ? (a)<<((q2)-(q1)) : (a)>>((q1)-(q2)))
/* the general operation between a in q1 format and b in q2 format
 returning the result in q3 format */
#define FADDG(a,b,q1,q2,q3) (FCONV(a,q1,q3)+FCONV(b,q2,q3))
#define FSUBG(a,b,q1,q2,q3) (FCONV(a,q1,q3)-FCONV(b,q2,q3))
#define FMULG(a,b,q1,q2,q3) FCONV((a)*(b), (q1)+(q2), q3)
#define FDIVG(a,b,q1,q2,q3) (FCONV(a, q1, (q2)+(q3))/(b))
/* convert to and from floating point */
#define TOFIX(d, q) ((int)( (d)*(float)(1<<(q)) ))
#define TOFLT(a, q) ( (float)(a) / (float)(1<<(q)) )

typedef enum{
	None,
	PWM,
	synth
}outputMode;

outputMode CurrentOMode =None;
outputMode NextOMode = None;

typedef struct{
	uint16_t ADC_VAL;
	uint16_t PWM_PR;
	uint16_t PWM_ARR;
	uint16_t Synth_PR;
	uint16_t Synth_ARR;
	uint8_t Keyboard;
}note;

#define NUMBER_OF_NOTES 28
#define NOTE_ACTIVE_MAX_ADC 3500
#define NOTE_ACTIVE_MIN_ADC 900
note notes[NUMBER_OF_NOTES];

void fill_notes_table()
{
	notes[0].ADC_VAL =  3027;
	notes[1].ADC_VAL =  2607;
	notes[2].ADC_VAL =  2291;
	notes[3].ADC_VAL =  2043;
	notes[4].ADC_VAL =  1841;
	notes[5].ADC_VAL =  1674;
	notes[6].ADC_VAL =  1535;
	notes[7].ADC_VAL =  1415;
	notes[8].ADC_VAL =  1311;
	notes[9].ADC_VAL =  1219;
	notes[10].ADC_VAL = 1140;
	notes[11].ADC_VAL = 1068;
	notes[12].ADC_VAL = 1005;
	notes[13].ADC_VAL = 949;
	notes[14].ADC_VAL = 800;
	notes[15].ADC_VAL = 3027;
	notes[16].ADC_VAL = 2607;
	notes[17].ADC_VAL = 2291;
	notes[18].ADC_VAL = 2043;
	notes[19].ADC_VAL = 1841;
	notes[20].ADC_VAL = 1674;
	notes[21].ADC_VAL = 1535;
	notes[22].ADC_VAL = 1415;
	notes[23].ADC_VAL = 1311;
	notes[24].ADC_VAL = 1219;
	notes[25].ADC_VAL = 1140;
	notes[26].ADC_VAL = 1068;
	notes[27].ADC_VAL = 1005;
	notes[0].PWM_PR = 20;
	notes[1].PWM_PR = 20;
	notes[2].PWM_PR = 20;
	notes[3].PWM_PR = 20;
	notes[4].PWM_PR = 20;
	notes[5].PWM_PR = 20;
	notes[6].PWM_PR = 20;
	notes[7].PWM_PR = 20;
	notes[8].PWM_PR = 20;
	notes[9].PWM_PR = 20;
	notes[10].PWM_PR = 20;
	notes[11].PWM_PR = 20;
	notes[12].PWM_PR = 20;
	notes[13].PWM_PR = 20;
	notes[14].PWM_PR = 20;
	notes[15].PWM_PR = 20;
	notes[16].PWM_PR = 20;
	notes[17].PWM_PR = 20;
	notes[18].PWM_PR = 20;
	notes[19].PWM_PR = 20;
	notes[20].PWM_PR = 20;
	notes[21].PWM_PR = 20;
	notes[22].PWM_PR = 20;
	notes[23].PWM_PR = 20;
	notes[24].PWM_PR = 20;
	notes[25].PWM_PR = 20;
	notes[26].PWM_PR = 20;
	notes[27].PWM_PR = 20;

	notes[0].PWM_ARR = 38636;
	notes[1].PWM_ARR = 36468;
	notes[2].PWM_ARR = 34421;
	notes[3].PWM_ARR = 32489;
	notes[4].PWM_ARR = 30665;
	notes[5].PWM_ARR = 28945;
	notes[6].PWM_ARR = 27320;
	notes[7].PWM_ARR = 25787;
	notes[8].PWM_ARR = 24339;
	notes[9].PWM_ARR = 22973;
	notes[10].PWM_ARR = 21684;
	notes[11].PWM_ARR = 20467;
	notes[12].PWM_ARR = 19318;
	notes[13].PWM_ARR = 18234;
	notes[14].PWM_ARR = 17211;
	notes[15].PWM_ARR = 16245;
	notes[16].PWM_ARR = 15333;
	notes[17].PWM_ARR = 14472;
	notes[18].PWM_ARR = 13660;
	notes[19].PWM_ARR = 12893;
	notes[20].PWM_ARR = 12170;
	notes[21].PWM_ARR = 11487;
	notes[22].PWM_ARR = 10842;
	notes[23].PWM_ARR = 10233;
	notes[24].PWM_ARR = 9659;
	notes[25].PWM_ARR = 9117;
	notes[26].PWM_ARR = 8605;
	notes[27].PWM_ARR = 8122;

	notes[0].Synth_PR = 1;
	notes[1].Synth_PR = 1;
	notes[2].Synth_PR = 1;
	notes[3].Synth_PR = 1;
	notes[4].Synth_PR = 1;
	notes[5].Synth_PR = 1;
	notes[6].Synth_PR = 1;
	notes[7].Synth_PR = 1;
	notes[8].Synth_PR = 1;
	notes[9].Synth_PR = 1;
	notes[10].Synth_PR = 1;
	notes[11].Synth_PR = 1;
	notes[12].Synth_PR = 1;
	notes[13].Synth_PR = 1;
	notes[14].Synth_PR = 1;
	notes[15].Synth_PR = 1;
	notes[16].Synth_PR = 1;
	notes[17].Synth_PR = 1;
	notes[18].Synth_PR = 1;
	notes[19].Synth_PR = 1;
	notes[20].Synth_PR = 1;
	notes[21].Synth_PR = 1;
	notes[22].Synth_PR = 1;
	notes[23].Synth_PR = 1;
	notes[24].Synth_PR = 1;
	notes[25].Synth_PR = 1;
	notes[26].Synth_PR = 1;
	notes[27].Synth_PR = 1;

	notes[0].Synth_ARR = 386;
	notes[1].Synth_ARR = 365;
	notes[2].Synth_ARR = 344;
	notes[3].Synth_ARR = 325;
	notes[4].Synth_ARR = 307;
	notes[5].Synth_ARR = 289;
	notes[6].Synth_ARR = 273;
	notes[7].Synth_ARR = 258;
	notes[8].Synth_ARR = 243;
	notes[9].Synth_ARR = 230;
	notes[10].Synth_ARR =217;
	notes[11].Synth_ARR =205;
	notes[12].Synth_ARR =193;
	notes[13].Synth_ARR =182;
	notes[14].Synth_ARR =172;
	notes[15].Synth_ARR =162;
	notes[16].Synth_ARR =153;
	notes[17].Synth_ARR =145;
	notes[18].Synth_ARR =137;
	notes[19].Synth_ARR =129;
	notes[20].Synth_ARR =122;
	notes[21].Synth_ARR =115;
	notes[22].Synth_ARR =108;
	notes[23].Synth_ARR =102;
	notes[24].Synth_ARR =97;
	notes[25].Synth_ARR =91;
	notes[26].Synth_ARR =86;
	notes[27].Synth_ARR =81;

	for (int i = 0; i < NUMBER_OF_NOTES; i++)
	{
		if(i<15) notes[i].Keyboard = 0;
		else notes[i].Keyboard = 1;
	}

}

uint8_t getActiveNote(uint16_t KeyboardL, uint16_t KeyboardH)
{
	for(int i =0; i< NUMBER_OF_NOTES; i++)
	{
		if ( (KeyboardL > NOTE_ACTIVE_MIN_ADC && KeyboardL < NOTE_ACTIVE_MAX_ADC) && notes[i].Keyboard == 0 )
		{
			uint16_t valuetocompare = KeyboardL;
			if(valuetocompare > notes[i].ADC_VAL) return i;
		}
		else if ((KeyboardH > NOTE_ACTIVE_MIN_ADC && KeyboardH < NOTE_ACTIVE_MAX_ADC) &&notes[i].Keyboard == 1 )
		{
			uint16_t valuetocompare = KeyboardH;
			if(valuetocompare > notes[i].ADC_VAL) return i;
		}
	}
	return NUMBER_OF_NOTES+1;
}
