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



enum keys{error0,error1,error2,error3,a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w,x,y,z,k1,k2,k3,k4,k5,k6,k7,k8,k9,k0,enter,esc,backspace,tab,space,minus,plus,capslock=57,f1=58,f2=59,f3=60,f4=61,f5=62,f6=63,f7=64,f8=65,pause=72,insert=73,home=74,pgup=75,delete=76,end=77,pgdown=78,right=79,left=80,down=81,up=82};

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

}

struct t_macro{
	uint8_t len;
	uint8_t * keyLst;
	//length len
	float * delayLst;
	//length len-1
	uint8_t modifiers;
	uint8_t repeatFlag:1;
	uint8_t noOtherKeysFlag:1;
	uint8_t simpleKeyFlag:1;
};

struct t_layout {
	struct t_macro keyBinds[keysInPad];
	char * layoutName;
	uint8_t joystickKeys[4];
	uint8_t dialKeys[2];

};


#endif /* KEYBOARDBINDS_H_ */
