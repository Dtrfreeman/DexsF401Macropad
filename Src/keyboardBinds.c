#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "keyboardBinds.h"
#include "main.h"

uint32_t charToInt(char * c, uint8_t * charsRead) {
	uint32_t total = 0;
	uint8_t curIndex = *charsRead;
	uint8_t charIndex = curIndex;
	uint16_t curMultiplier = 1;
	charIndex++;
	while ((c[charIndex] >= '0') && (c[charIndex] <= '9')) {
		charIndex++;
		curMultiplier *= 10;
	}

	while ((c[curIndex] >= '0') && (c[curIndex] <= '9')) {
		total += (c[curIndex] - '0')*curMultiplier;
		curMultiplier /= 10;

		curIndex++;
	}
	*charsRead = curIndex;
	return(total);

}

struct t_macro layoutCodeParser(char * lCode,uint8_t * pReadHead) {
	uint8_t curIndex = *pReadHead;
	char curChar;

	uint8_t lenOfMacro = 1;
	while ((lCode[curIndex] != ';') && (lCode[curIndex] != 0)) {
		curIndex++;
		if (lCode[curIndex] == ',') { lenOfMacro++; }
	}

	curIndex = 0;
	while (lCode[curIndex] == ' ') { curIndex++; }
	//to skip over whitespace
	struct t_macro macro = { lenOfMacro,0,0,malloc(lenOfMacro),lenOfMacro>1?malloc(lenOfMacro - 1):0,0/*mod*/,0,0,0,0,0,0};
	uint16_t curTotalDelay=0;
	uint8_t curKeyPos = 0;

	do {
		if ((curKeyPos != 0) && lCode[curIndex] == '.') {
			curIndex++;
			curTotalDelay+=charToInt(lCode, &curIndex);
			macro.delayLst[curKeyPos] = curTotalDelay;
			curIndex++;//skips over the comma
			macro.simpleKeyFlag = 0;
		}


		curChar = lCode[curIndex];
		if (curChar != 'k'&&curChar != 'm') {
			return(macro);
		}
		if (curChar == 'm') {
			macro.forMediaFlag = 1;
		}
		curIndex++;
		curChar = lCode[curIndex];
		switch (curChar) {
		case 'c':
			curIndex++;
			curChar = lCode[curIndex];
			macro.keyLst[curKeyPos] = 4 + ((curChar >= 'A') ? curChar - 'A' : curChar - 'a');
			curKeyPos++;
			curIndex++;
			break;
		case 'n':
			curIndex++;
			curChar = lCode[curIndex];
			macro.keyLst[curKeyPos] = curChar - '0';
			curKeyPos++;
			curIndex++;
			break;
		case 'u':
			curIndex++;
			macro.keyLst[curKeyPos] = charToInt(lCode, &curIndex);
			curKeyPos++;
			break;
		default:
			return(macro);
			break;
		}

		while (lCode[curIndex] == ' ') { curIndex++; }
		//skip whitespace

		curChar = lCode[curIndex];
		if (curChar == 'r') {
			macro.repeatFlag = 1;
			curIndex++;
			curChar = lCode[curIndex];
		}

		if (curChar == 'q') {//for unique
			macro.noOtherKeysFlag = 1;
			curIndex++;
			curChar = lCode[curIndex];
		}

	} while ((curChar != ';') && (curChar != ':') && (curKeyPos < lenOfMacro));
	macro.completedFlag = 1;
	*pReadHead += curIndex-1;
	return(macro);

}

struct t_layout createLayout(char * sKR1, char * sKR2, char * sKR3, char * sKR4, char * playoutName, char * joystickKeys, char * dialKeys) {

	struct t_layout layout;
	//layout.layoutName =malloc(strlen(playoutName));
	//memcpy(layout.layoutName,playoutName, strlen(playoutName)+1);
	strcpy_s(layout.layoutName, 20, playoutName);
	uint8_t readHeadInd = 0;
	uint8_t neededStrLength = strlen(sKR1) + strlen(sKR2) + strlen(sKR3) + strlen(sKR4) + 5;
	char * totalButtonStr = malloc(neededStrLength);
	memset(totalButtonStr, 0, neededStrLength);
	strcat_s(totalButtonStr, neededStrLength,sKR1);
	strcat_s(totalButtonStr, neededStrLength,sKR2);
	strcat_s(totalButtonStr, neededStrLength,sKR3);
	strcat_s(totalButtonStr, neededStrLength,sKR4);

	for (uint8_t i = 0; i < keysInPad; i++) {
		while ((totalButtonStr[readHeadInd] != 'k') && (totalButtonStr[readHeadInd] != 'm')) { readHeadInd++; }
		layout.keyBinds[i] = layoutCodeParser(totalButtonStr + readHeadInd,&readHeadInd);
	}
	readHeadInd = 0;
	for (uint8_t i = 0; i < 4; i++) {
		while ((joystickKeys[readHeadInd] != 'k') && (joystickKeys[readHeadInd] != 'm')) { readHeadInd++; }
		layout.joystickKeys[i] = layoutCodeParser(joystickKeys + readHeadInd,&readHeadInd);
	}
	readHeadInd = 0;
	for (uint8_t i = 0; i < 2; i++) {
		while ((dialKeys[readHeadInd] != 'k') && (dialKeys[readHeadInd] != 'm')) { readHeadInd++; }
		layout.dialKeys[i] = layoutCodeParser(dialKeys + readHeadInd, &readHeadInd);
	}


		free(totalButtonStr);
		return(layout);
}
//K/M = keyboard/media
//C/N/U = char/numerical/uid(direct)
//,=next key .=add delay (must be a delay between keys, can be 0)
//https://www.win.tue.nl/~aeb/linux/kbd/scancodes-14.html


#define bufferSizePerKey 100
void macro2str(struct t_macro * macro,char * strOutLoc,uint16_t maxLen) {
	char str [bufferSizePerKey];

	snprintf(str, bufferSizePerKey, "	len:%u index:%u mod:%u flags:r%u ,n%u ,s%u ,m%u\r\n", macro->len, macro->index, macro->modifiers, macro->repeatFlag, macro->noOtherKeysFlag, macro->simpleKeyFlag, macro->forMediaFlag);
	strcat_s(strOutLoc, maxLen, str);
	memset(str, 0, bufferSizePerKey);

	for (uint8_t i = 0; i < macro->len; i++) {
		snprintf(str, bufferSizePerKey, "		Key:%u, Delay:%lu   ", macro->keyLst[i], (i > 0) ? (uint32_t)macro->delayLst[i - 1] : 0);
		strcat_s(strOutLoc, maxLen, str);
		memset(str, 0, bufferSizePerKey);
	}



}

void inputmode2Str(struct t_macro pMacArry[],uint8_t sizeOfMacroLst,char * cOut,uint16_t maxLen) {
	uint16_t curWriteHead = 0;
	uint16_t curLen = 0;
	uint16_t bufLen = 0;

	char buf[bufferSizePerKey];

	for (uint8_t i = 0; i < sizeOfMacroLst; i++) {
		snprintf(buf, bufferSizePerKey*10, "key:%u\r\n", i);
		strcat_s(cOut, maxLen, buf);
		memset(buf, 0, bufferSizePerKey);
		macro2str(&pMacArry[i], cOut,maxLen);
		snprintf(buf, bufferSizePerKey*10, "key %u over\r\n", i);
		strcat_s(cOut, maxLen, buf);
		memset(buf, 0, bufferSizePerKey);
	}


}

#define maxLen (bufferSizePerKey*24)

void layout2str(struct t_layout * pLayout) {

	uint16_t curWriteHead = 0;
	uint16_t curLen = 0;

	uint16_t bufLen = 0;


	char output[maxLen];
	char buf[bufferSizePerKey];
	snprintf(output, maxLen,"Name: %s\r\nBindings: ", pLayout->layoutName);
	curLen = strlen(output);
	snprintf(output+ curLen, maxLen -curLen, "	Keypad:\r\n ");

	inputmode2Str(pLayout->keyBinds, keysInPad, output, maxLen);

	curLen = strlen(output);
	snprintf(output + curLen, maxLen - curLen, " Joystick:\r\n ");

	inputmode2Str(pLayout->joystickKeys, 4, output, maxLen);


	curLen = strlen(output);
	snprintf(output + curLen, maxLen - curLen, "Wheel:\r\n");

	inputmode2Str(pLayout->dialKeys, 2, output, maxLen);
	//output = dynamicConcat(output, &maxLen,
		//inputmode2Str(pLayout->dialKeys, 2));
	puts(output);
}
