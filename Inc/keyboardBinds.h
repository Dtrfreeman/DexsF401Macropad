/*
 * keyboardBinds.h
 *
 *  Created on: 30 Dec 2021
 *      Author: User
 */

#ifndef KEYBOARDBINDS_H_
#define KEYBOARDBINDS_H_

#define gui (1<<7)
#define alt (1<<6)
#define shift (1<<5)
#define ctrl (1<<4)
#define lgui (1<<3)
#define lalt (1<<2)
#define lshift (1<<1)
#define lctrl 1

#define kEnter 40
#define kEsc 41
#define kBackspace 42
#define kTab 43

#define keysInPad 12

#define usbKeySize 3

//enum keys{error0,error1,error2,error3,a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w,x,y,z,k1,k2,k3,k4,k5,k6,k7,k8,k9,k0,enter,esc,backspace,tab,space,minus,plus,capslock=57,f1=58,f2=59,f3=60,f4=61,f5=62,f6=63,f7=64,f8=65,pause=72,insert=73,home=74,pgup=75,delete=76,end=77,pgdown=78,right=79,left=80,down=81,up=82};
/*
uint8_t atoUID(char curChar){

	if((curChar>='A')&&(curChar<='Z')){
		curChar=(curChar-'A')+'a';
	}

	if((curChar>='a')&&(curChar<='z')){

		return((curChar-'a')+4);
	}
	if((curChar>='0')&&(curChar<='9')){
		if(curChar==0){return(39);}
		return((curChar-'1')+30);
	}

	return(0);

}*/


#define keysInPad 12


struct t_macro {
	uint8_t len : 4;
	uint8_t index : 4;
	uint16_t ticksRunning;
	uint8_t * keyLst;
	//length len
	uint16_t * delayLst;
	//length len-1
	uint8_t modifiers;
	uint8_t repeatFlag : 1;
	uint8_t noOtherKeysFlag : 1;
	uint8_t simpleKeyFlag : 1;
	uint8_t forMediaFlag : 1;
	uint8_t completedFlag : 1;
	uint8_t currentlyRunning:1;
};

struct t_layout {
	char layoutName[20];
	struct t_macro keyBinds[keysInPad];
	struct t_macro joystickKeys[4];
	struct t_macro dialKeys[2];

};

uint32_t charToInt(char * c, uint8_t * charsRead);


struct t_macro layoutCodeParser(char * lCode,uint8_t * pReadHead,uint8_t maxLen) ;

struct t_layout * createLayout(char * sKR1, char * sKR2, char * sKR3, char * sKR4, char * playoutName, char * joystickKeys, char * dialKeys);

void macro2str(struct t_macro * macro,char * strOutLoc,uint16_t maxLen);

void inputmode2Str(struct t_macro pMacArry[],uint8_t sizeOfMacroLst,char * cOut,uint16_t maxLen);

void layout2str(struct t_layout * pLayout);
#endif /* KEYBOARDBINDS_H_ */
