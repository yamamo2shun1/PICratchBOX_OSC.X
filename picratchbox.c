/*
 * Copylight (C) 2009, Shunichi Yamamoto, tkrworks.net
 *
 * This file is used the copied code from picnome.c in PICnome project files.
 *
 * Copylight (C) 2011, Shunichi Yamamoto, tkrworks.net
 *
 * This file is part of PICratchBOX_OSC.
 * PICratchBOX is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option ) any later version.
 *
 * PICratchBOX is distributed in the hope that it will be useful,
 * but WITHIOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.   See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with PICnome. if not, see <http:/www.gnu.org/licenses/>.
 *
 * picratchbox.c,v.1.0.06 2012/01/17
 */

#include "picratchbox.h"

#pragma code

int main(void)
{
  CLKDIV = 0;
  TRISA = 0x0003;
#ifdef MULTI_PLEXER
  //AD1PCFGL = 0xFFF0;//AN0,AN1,AN2,AN9
  TRISB = 0x00A3;//0000 0000 1010 0011
#else
  //AD1PCFGL = 0xFFC0;//AN0,AN1,AN2,AN3,AN4,AN5
  TRISB = 0x00AF;//0000 0000 1010 1111
#endif
  ODCBbits.ODB9 = 1;
  ODCBbits.ODB4 = 1;
  ODCAbits.ODA4 = 1;
  ODCBbits.ODB13 = 1;
  ODCBbits.ODB14 = 1;

  WORD pll_startup_counter = 600;
  CLKDIVbits.PLLEN = 1;
  while(pll_startup_counter--);

  RPOR3bits.RP7R = 7;// SDO PIN_B7
  RPOR4bits.RP8R = 8;// SCK PIN_B8

  //sy SPI1CON1 = 0x053f;//sy 16mhz   /* Initalizes the spi module */
  //sy SPI1CON1 = 0x053b;//sy 8mhz   /* Initalizes the spi module */
  SPI1CON1 = 0x053e;//sy 4mhz   /* Initalizes the spi module */
  //sy SPI1CON1 = 0x053a;//sy 2mhz   /* Initalizes the spi module */
  //sy SPI1CON1 = 0x0536;//sy 1mhz   /* Initalizes the spi module */
  SPI1CON2 = 0x0000;
  SPI1STATbits.SPIEN = 1;

  IFS0bits.SPI1IF = 0; /* Clear IF bit */
  IPC2bits.SPI1IP = 0; /* Assign interrupt priority */
  IEC0bits.SPI1IE = 0; /* Interrupt Enable/Disable bit */

  // A/D Conversion Interrupt Intialize
  PR3 = 4;//sy 311; // 5msec
  T3CON = 0b1000000000110000;
  AD1CON1 = 0x8044;//1000 0000 0100 0100
  AD1CON2 = 0x042C;//0000 0100 0010 1100 //sy 0x0414;
  AD1CON3 = 0x1F02;//0001 1111 0000 0010 //sy 0x1F05;
  AD1CHS =  0x0000;

#ifdef MULTI_PLEXER
  AD1PCFG = 0xFFF0;//1111 1111 1111 0000
  AD1CSSL = 0x000F;//0000 0000 0000 1111
#else
  AD1PCFG = 0xFFC0;
  AD1CSSL = 0x003F;
#endif

  IEC0bits.AD1IE = 1;

  gAdcEnableState = 0x0000;

#ifdef MULTI_PLEXER
  scanID = 0;
  scanCount = 0;
  MP_A = (scanID & 0x01);
  MP_B = ((scanID >> 1) & 0x01);
  MP_C = ((scanID >> 2) & 0x01);
#endif

  countChk = 0;
  for(i = 0; i < NUM_ADC_PINS; i++)
  {
    adcSendFlag[i] = FALSE;
    anlg1[i] = 0;
    anlg0[i] = 0;
  }

  DataEEInit();
  dataEEFlags.val = 0;

  xfader_curve = DataEERead(xfaddr);
  volfader_curve[0] = DataEERead(vf0addr);
  volfader_curve[1] = DataEERead(vf1addr);
  if(xfader_curve > 7)
    xfader_curve = 4;
  if(volfader_curve[0] > 7)
    volfader_curve[0] = 4;
  if(volfader_curve[1] > 7)
    volfader_curve[1] = 4;

#if 0//test
  xrev = DataEERead(revxaddr);
  v0rev = DataEERead(rev0addr);
  v1rev = DataEERead(rev1addr);

  if(xrev > 1)
    xrev  = 0;
  if(v0rev > 1)
    v0rev  = 0;
  if(v1rev > 1)
    v1rev  = 0;
#endif//test

  delayMs(100);

  SR_A = 1;
  for(i = 0; i < 8; i++)
  {
    led_data[i] = 0;
    SR_CLK = 1;
    SR_CLK = 0;
  }
  buttonInit();
  initLedDriver();

  USBDeviceInit();

#if defined(USB_INTERRUPT)
  USBDeviceAttach();
#endif

  i = 1, j = 1;
  while(1)
  {
#if defined(USB_POLLING)
    USBDeviceTasks();
#endif

    if((USBDeviceState >= CONFIGURED_STATE) && (USBSuspendControl == 0))
    {
      sendOscMsgPress();
      sendOscMsgAdc();
      receiveOscMsgs();
      CDCTxService();
    }
  }
}

/***********************************/
/*                                 */
/*    Functions for Max7219CNG     */
/*                                 */
/***********************************/
void initLedDriver(void)
{
  sendSpiLED(0x0B, 0x03); // Scan Limit full range
  sendSpiLED(0x0C, 0x01); // Shutdown Normal Operation
  sendSpiLED(0x0F, 0x00); // Display Test Off

 // print startup pattern
  sendSpiLED(0x0A, 0x0F); // Max Intensity 0x00[min] - 0x0F[max]
  sendSpiLED(1, 1 + 128);
  sendSpiLED(2, 0 + 16);
  sendSpiLED(3, 0);
  sendSpiLED(4, 0);
  sendSpiLED(5, 0);
  sendSpiLED(6, 0);
  sendSpiLED(7, 0);
  sendSpiLED(8, 0);
  for(i = 0; i < 64; i++)
  {
    sendSpiLED(0x0A, (64 - i) / 4); // set to max intensity
    delayMs(1);
  }
  sendSpiLED(0x0A, 0x0F); // Max Intensity 0x00[min] - 0x0F[max]
  sendSpiLED(1, 2 + 64);
  sendSpiLED(2, 3 + 32);
  sendSpiLED(3, 0);
  sendSpiLED(4, 0);
  sendSpiLED(5, 0);
  sendSpiLED(6, 0);
  sendSpiLED(7, 0);
  sendSpiLED(8, 0);
  for(i = 0; i < 64; i++)
  {
    sendSpiLED(0x0A, (64 - i) / 4); // set to max intensity
    delayMs(1);
  }
  sendSpiLED(0x0A, 0x0F); // Max Intensity 0x00[min] - 0x0F[max]
  sendSpiLED(1, 4 + 32);
  sendSpiLED(2, 4 + 64);
  sendSpiLED(3, 7);
  sendSpiLED(4, 0);
  sendSpiLED(5, 0);
  sendSpiLED(6, 0);
  sendSpiLED(7, 0);
  sendSpiLED(8, 0);
  for(i = 0; i < 64; i++)
  {
    sendSpiLED(0x0A, (64 - i) / 4); // set to max intensity
    delayMs(1);
  }
  sendSpiLED(0x0A, 0x0F); // Max Intensity 0x00[min] - 0x0F[max]
  sendSpiLED(1, 0 + 16);
  sendSpiLED(2, 0 + 128);
  sendSpiLED(3, 0);
  sendSpiLED(4, 0);
  sendSpiLED(5, 0);
  sendSpiLED(6, 0);
  sendSpiLED(7, 0);
  sendSpiLED(8, 0);
  for(i = 0; i < 64; i++)
  {
    sendSpiLED(0x0A, (64 - i) / 4); // set to max intensity
    delayMs(1);
  }
  intensity = DataEERead(intensityaddr);
  if(intensity > 15)
    sendSpiLED(0x0A, 15);
  else
    sendSpiLED(0x0A, intensity);
  for(i = 1; i < 9; i++)
    sendSpiLED(i, 0x00);
}

void sendSpiLED(BYTE msb, BYTE lsb)
{
  spibits = (0xFF00 & ((0x00FF & (WORD)msb) << 8)) + (0x00FF & (WORD)lsb);
  LDD_LOAD = 0;
  while(SPI1STATbits.SPITBF);
  SPI1BUF = spibits;
  delayUs(1);
  LDD_LOAD = 1;
}

/***********************************/
/*                                 */
/*    Function for OSC Msgs Rcv    */
/*                                 */
/***********************************/
void receiveOscMsgs(void)
{
  if(getsUSBUSART(string, 15))
  {
    if(string[0] == 'l' && (string[1] == '0' || string[1] == '1')) // led
    {
      y = my_atoi(string[3]);
      if(y >= 8)
        return;
      x = my_atoi(string[2]);
      state = my_atoi(string[1]);

      if(y < 4)
      {
        if(state)
          led_data[3 - x] |= (1 << y);
        else
          led_data[3 - x] &= ~(1 << y);
        sendSpiLED((3 - x) + 1, led_data[3 - x]);
      }
      else
      {
        if(state)
          led_data[2 - x] |= (1 << y);
        else
          led_data[2 - x] &= ~(1 << y);
        sendSpiLED((2 - x) + 1, led_data[2 - x]);
      }
    }
    else if(string[0] == 'l' && string[1] == 'r') // led_col
    {
      BYTE column, data, data2;
      ch = strtok(string, space);
      ch = strtok(0, space);
      column = atoi(ch);
      ch = strtok(0, space);
      data = atoi(ch);

      data2 = 0x00;
      if((data & 0x01) == 0x01)
        data2 |= 0x08;
      if((data & 0x02) == 0x02)
        data2 |= 0x04;
      if((data & 0x04) == 0x04)
        data2 |= 0x02;
      if((data & 0x08) == 0x08)
        data2 |= 0x01;
      if(column > 3)
        data2 >>= 1;

      if(firstRun == TRUE)
      {
        for(i = 0; i < 8; i++)
        {
          led_data[i] = 0;
          sendSpiLED(i + 1, led_data[i]);
        }
        firstRun = FALSE;
      }
      for(i = 0; i < 8; i++)
      {
        if(data2 & (1 << i))
          led_data[i] |= 1 << column;
        else
          led_data[i] &= ~(1 << column);

        sendSpiLED(i + 1, led_data[i]);
      }
    }
    else if(string[0] == 'l' && string[1] == 'c') // led_row
    {
      BYTE row, data, data2;
      ch = strtok(string, space);
      ch = strtok(0, space);
      row = atoi(ch);
      ch = strtok(0, space);
      data = atoi(ch);

      if(firstRun == TRUE)
      {
        for(i = 0; i < 8; i++)
        {
          led_data[3 - i] = 0;
          sendSpiLED(3 - i + 1, led_data[3 - i]);
        }
        firstRun = FALSE;
      }
      if(row == 1 || row == 2)
      {
        data2 = data & 0xF0;
        led_data[3 - row - 1] = (led_data[3 - row - 1] & 0x0F) | data2;
        sendSpiLED(3 - row, led_data[3 - row - 1]);
        delayUs(1);
        data = data & 0x0F;
      }
      else if(row == 3)
      {
        data = data & 0x0F;
      }
      led_data[3 - row] = data;
      sendSpiLED(3 - row + 1, led_data[3 - row]);
    }
    else if(string[0] == 'a' && string[1] == 'e') // adc_enable
    {
      BYTE pin, state;
      ch = strtok(string, space);
      ch = strtok(0, space);
      pin = atoi(ch);
      ch = strtok(0, space);
      state = atoi(ch);
      if(state)
        enableAdc(pin);
      else
        disableAdc(pin);
    }
    else if(string[0] == 'f' && string[1] == 'r')
    {
      BYTE pin, toggle;
      ch = strtok(string, space);
      ch = strtok(0, space);
      pin = atoi(ch);
      ch = strtok(0, space);
      state = atoi(ch);
      switch(pin)
      {
        case 0:
          DataEEWrite(toggle, revxaddr);
          break;
        case 1:
          DataEEWrite(toggle, rev0addr);
          break;
        case 2:
          DataEEWrite(toggle, rev1addr);
          break;
      }
    }
    else if(string[0] == 'c' && string[1] == 's')
    {
      BYTE pin, curve;
      ch = strtok(string, space);
      ch = strtok(0, space);
      pin = atoi(ch);
      ch = strtok(0, space);
      curve = atoi(ch);
      switch(pin)
      {
        case 0:
          xfader_curve = curve;
          DataEEWrite(curve, xfaddr);
          break;
        case 1:
          volfader_curve[0] = curve;
          DataEEWrite(curve, vf0addr);
          break;
        case 2:
          volfader_curve[1] = curve;
          DataEEWrite(curve, vf1addr);
          break;
      }
    }
    else if(string[0] == 'c' && string[1] == 'g')
    {
      BYTE pin;
      ch = strtok(string, space);
      ch = strtok(0, space);
      pin = atoi(ch);
      sendmsg[0] = 'c';
      sendmsg[1] = 'g';
      sendmsg[2] = pin;
      switch(pin)
      {
        case 0:
          xfader_curve = DataEERead(xfaddr);
          if(xfader_curve > 7)
            xfader_curve = 4;
          sendmsg[3] = xfader_curve;
          break;
        case 1:
          volfader_curve[0] = DataEERead(vf0addr);
          if(volfader_curve[0] > 7)
            volfader_curve[0] = 4;
          sendmsg[3] = volfader_curve[0];
          break;
        case 2:
          volfader_curve[1] = DataEERead(vf1addr);
          if(volfader_curve[1] > 7)
            volfader_curve[1] = 4;
          sendmsg[3] = volfader_curve[1];
          break;
      }
      if(mUSBUSARTIsTxTrfReady())
        mUSBUSARTTxRam(sendmsg, 4);
      CDCTxService();
      delayUs(2);
    }
    else if(string[0] == 'i') // intensity
    {
      ch = strtok(string, space);
      ch = strtok(0, space);
      sendSpiLED(0x0A, atoi(ch));
    }
    else if(string[0] == 't') // test
    {
      BYTE state;
      ch = strtok(string, space);
      ch = strtok(0, space);
      state = atoi(ch);
      sendSpiLED(15, state);
    }
    else if(string[0] == 's') // shutdown
    {
      BYTE state;
      ch = strtok(string, space);
      ch = strtok(0, space);
      state = atoi(ch);
      sendSpiLED(12, state);
    }
    else if(string[0] == 'f') // firmware
    {
      sendmsg[0] = 'f';
      sendmsg[1] = 10;
      sendmsg[2] = 6;
      if(mUSBUSARTIsTxTrfReady())
        mUSBUSARTTxRam(sendmsg, 3);
      CDCTxService();
      delayUs(2);
    }
  }
}

/***********************************/
/*                                 */
/*  Functions for Button Handling  */
/*                                 */
/***********************************/
void buttonInit(void)
{
  for(i = 0; i < 8; i++)
  {
    btnCurrent[i] = 0x00;
    btnLast[i] = 0x00;
    btnState[i] = 0x00;

    for(j = 0; j < 8; j++)
      btnDebounceCount[i][j] = 0;
  }
}

BOOL buttonCheck(int row, int index)
{
  flag = FALSE;

  if(((btnCurrent[row] ^ btnLast[row]) & (1 << index)) && ((btnCurrent[row] ^ btnState[row]) & (1 << index)))
    btnDebounceCount[row][index] = 0;
  else if (((btnCurrent[row] ^ btnLast[row]) & (1 << index)) == 0 && ((btnCurrent[row] ^ btnState[row]) & (1 << index)))
  {
    if(btnDebounceCount[row][index] < 4 && ++btnDebounceCount[row][index] == 4)
    {
      //debug sendSpiLED2(row + 1, (BYTE)(btnCurrent[row] & 0x00FF), row + 1, (BYTE)((btnCurrent[row] >> 8) & 0x00FF));
      if(btnCurrent[row] & (1 << index))
        btnState[row] |= (1 << index);
      else
        btnState[row] &= ~(1 << index);
      flag = TRUE;
    }
  }
  return flag;
}

void sendOscMsgPress(void)
{
  if(start_row == 0)
  {
    SR_A = 0;
    SR_CLK = 1;
    SR_A = 1;
  }

  for(i = start_row; i < start_row + 4; i++)
  {
    SR_CLK = 1;
    for(k = 0; k < 5; k++)
      asm("NOP");
    SR_CLK = 0;

    btnLast[i] = btnCurrent[i];

    SR_SL = 0;
    delayUs(1);
    SR_SL = 1;

    for(j = 0; j < 8; j++)
    {
      if(SR_QH)
        btnCurrent[i] &= ~(1 << j);
      else
        btnCurrent[i] |= (1 << j);

      SR_CLK2 = 1;
      for(k = 0; k < 2; k++)
        asm("NOP");
      SR_CLK2 = 0;

      if(buttonCheck(i, j))
      {
        if(btnCurrent[i] & (1 << j))
          sendmsg[msg_index] = 'p';
        else
          sendmsg[msg_index] = 'r';
        //sy sendmsg[msg_index + 1] = (i << 4) + j;// i = y, j = x
        if(j < 4)
          sendmsg[msg_index + 1] = (j << 4) + (3 - i);// i = y, j = x
        else
          sendmsg[msg_index + 1] = (j << 4) + (2 - i);// i = y, j = x
        msg_index += 2;
      }
    }
  }

  if(msg_index > 0 && mUSBUSARTIsTxTrfReady())
  {
    mUSBUSARTTxRam(sendmsg, msg_index);
    msg_index = 0;
  }
  CDCTxService();
  delayUs(2);
}

/***********************************/
/*                                 */
/*  Functions for A/D Conversion   */
/*                                 */
/***********************************/
void enableAdc(int port)
{
  if(port >= NUM_ADC_PINS)
    return;

#ifdef MULTI_PLEXER
  if((gAdcEnableState & 0x07FF) == 0)
#else
  if((gAdcEnableState & 0x3F) == 0)
#endif
    enableAdcFlag = TRUE;

  gAdcEnableState |= ((WORD)1 << port);
  enableAdcNum++;
}

void disableAdc(int port)
{
  if(port >= NUM_ADC_PINS)
    return;

  gAdcEnableState &= ~((WORD)1 << port);

#ifdef MULTI_PLEXER
  if((gAdcEnableState & 0x07FF) == 0)
#else
  if((gAdcEnableState & 0x3F) == 0)
#endif
    enableAdcFlag = FALSE;
  enableAdcNum--;
}

void __attribute__((interrupt, auto_psv)) _ADC1Interrupt(void)
{
  IFS0bits.AD1IF = 0;

  MP_A = (scanID & 0x01);
  MP_B = ((scanID >> 1) & 0x01);
  MP_C = ((scanID >> 2) & 0x01);

#ifdef MULTI_PLEXER
  anlg[countChk][0] = ((ADC1BUF0 + ADC1BUF4 + ADC1BUF8) / 3);
  anlg[countChk][1] = ((ADC1BUF1 + ADC1BUF5 + ADC1BUF9) / 3);
  anlg[countChk][2] = ((ADC1BUF2 + ADC1BUF6 + ADC1BUFA) / 3);
  if(scanCount > 2)//output noize if scanCount is less than 1.
  {
    anlg[countChk2][scanID + 3] = ((ADC1BUF3 + ADC1BUF7 + ADC1BUFB) / 3);
    countChk2++;
  }

  scanCount++;
  if(scanCount > 2 + VOLUME_CHK_NUM)
  {
    countChk2 = 0;
    scanCount = 0;
    scanID++;
    if(scanID == NUM_MP_PINS)
      scanID = 0;
  }
#else
  anlg[countChk][0] = ((ADC1BUF0 + ADC1BUF6) / 2);
  anlg[countChk][1] = ((ADC1BUF1 + ADC1BUF7) / 2);
  anlg[countChk][2] = ((ADC1BUF2 + ADC1BUF8) / 2);
  anlg[countChk][3] = ((ADC1BUF3 + ADC1BUF9) / 2);
  anlg[countChk][4] = ((ADC1BUF4 + ADC1BUFA) / 2);
  anlg[countChk][5] = ((ADC1BUF5 + ADC1BUFB) / 2);
#endif

  countChk++;
  if(countChk == FADER_CHK_NUM)
    countChk = 0;

  //test for(p = 0; p < NUM_ADC_PINS; p++)
  for(p = 0; p < 3; p++)
  {
    if(countChk == 0 && (gAdcEnableState & ((WORD)0x0001 << p)) == ((WORD)0x0001 << p))
    {
      WORD sum = 0;
      for(q = 0; q < FADER_CHK_NUM; q++)
        sum += anlg[q][p];
        anlg1[p] = sum / (WORD)FADER_CHK_NUM;
      if(abs(anlg1[p] - anlg0[p]) > 4)
      {
        adcSendFlag[p] = TRUE;
        anlg0[p] = anlg1[p];
      }
    }
  }
  for(p = 3; p < NUM_ADC_PINS; p++)
  {
    if(countChk2 == 0 && (gAdcEnableState & ((WORD)0x0001 << p)) == ((WORD)0x0001 << p))
    {
      WORD sum = 0;
      for(q = 0; q < VOLUME_CHK_NUM; q++)
        sum += anlg[q][p];
        anlg1[p] = sum / (WORD)VOLUME_CHK_NUM;
      if(abs(anlg1[p] - anlg0[p]) > 4)
      {
        adcSendFlag[p] = TRUE;
        anlg0[p] = anlg1[p];
      }
    }
  }
}

void sendOscMsgAdc(void)
{
  for(i = 0; i < NUM_ADC_PINS; i++)
  {
    if((gAdcEnableState & ((DWORD)0x0001 << i)) == ((DWORD)0x0001 << i) && adcSendFlag[i])
    {
      sendmsg2[msg_index2] = 'a';
      sendmsg2[msg_index2 + 1] = (i << 4) + ((anlg1[i] & 0x0300) >> 8);
      sendmsg2[msg_index2 + 2] = (anlg1[i] & 0x00FF);
      msg_index2 += 3;
      adcSendFlag[i] = FALSE;
    }
  }
  if(msg_index2 >= 3  && mUSBUSARTIsTxTrfReady())
  {
    mUSBUSARTTxRam(sendmsg2, msg_index2);
    msg_index2 = 0;
    CDCTxService();
  }
  delayUs(1);
}

void delayUs(WORD usec)
{
  usec = (WORD)(CLOCK * usec) / 10;
  while(usec)
  {
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    usec--;
  }
}

void delayMs(WORD msec)
{
  for(k = 0; k < msec; k++)
    delayUs(1000);
}

int my_atoi(char s)
{
  int result = 0;

  if (s >= '0' && s <= '9')
    result = (result << 4) + (s - '0');
  else
    result = (result << 4) + (s - 'A' + 10);
  return result;
}

// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
void USBCBSuspend(void)
{
	//Example power saving code.  Insert appropriate code here for the desired
	//application behavior.  If the microcontroller will be put to sleep, a
	//process similar to that shown below may be used:

	//ConfigureIOPinsForLowPower();
	//SaveStateOfAllInterruptEnableBits();
	//DisableAllInterruptEnableBits();
	//EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();	//should enable at least USBActivityIF as a wake source
	//Sleep();
	//RestoreStateOfAllPreviouslySavedInterruptEnableBits();	//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.
	//RestoreIOPinsToNormal();									//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

	//IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is
	//cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause
	//things to not work as intended.


    #if defined(__C30__)
    #if 0
        U1EIR = 0xFFFF;
        U1IR = 0xFFFF;
        U1OTGIR = 0xFFFF;
        IFS5bits.USB1IF = 0;
        IEC5bits.USB1IE = 1;
        U1OTGIEbits.ACTVIE = 1;
        U1OTGIRbits.ACTVIF = 1;
        Sleep();
    #endif
    #endif
}

/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *					suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *					mode, the host may wake the device back up by sending non-
 *					idle state signalling.
 *
 *					This call back is invoked when a wakeup from USB suspend
 *					is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend(void)
{
	// If clock switching or other power savings measures were taken when
	// executing the USBCBSuspend() function, now would be a good time to
	// switch back to normal full power run mode conditions.  The host allows
	// a few milliseconds of wakeup time, after which the device must be
	// fully back to normal, and capable of receiving and processing USB
	// packets.  In order to do this, the USB module must receive proper
	// clocking (IE: 48MHz clock must be available to SIE for full speed USB
	// operation).
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 *******************************************************************/
void USBCB_SOF_Handler(void)
{
#if 0//sy
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.

    //This is reverse logic since the pushbutton is active low
    if(buttonPressed == sw2)
    {
        if(buttonCount != 0)
        {
            buttonCount--;
        }
        else
        {
            //This is reverse logic since the pushbutton is active low
            buttonPressed = !sw2;

            //Wait 100ms before the next press can be generated
            buttonCount = 100;
        }
    }
    else
    {
        if(buttonCount != 0)
        {
            buttonCount--;
        }
    }
#endif//sy
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler(void)
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

	// Typically, user firmware does not need to do anything special
	// if a USB error occurs.  For example, if the host sends an OUT
	// packet to your device, but the packet gets corrupted (ex:
	// because of a bad connection, or the user unplugs the
	// USB cable during the transmission) this will typically set
	// one or more USB error interrupt flags.  Nothing specific
	// needs to be done however, since the SIE will automatically
	// send a "NAK" packet to the host.  In response to this, the
	// host will normally retry to send the packet again, and no
	// data loss occurs.  The system will typically recover
	// automatically, without the need for application firmware
	// intervention.

	// Nevertheless, this callback function is provided, such as
	// for debugging purposes.
}

/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 * 					firmware must process the request and respond
 *					appropriately to fulfill the request.  Some of
 *					the SETUP packets will be for standard
 *					USB "chapter 9" (as in, fulfilling chapter 9 of
 *					the official USB specifications) requests, while
 *					others may be specific to the USB device class
 *					that is being implemented.  For example, a HID
 *					class device needs to be able to respond to
 *					"GET REPORT" type of requests.  This
 *					is not a standard USB chapter 9 request, and
 *					therefore not handled by usb_device.c.  Instead
 *					this request should be handled by class specific
 *					firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *******************************************************************/
void USBCBCheckOtherReq(void)
{
    USBCheckCDCRequest();
}//end

/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *					called when a SETUP, bRequest: SET_DESCRIPTOR request
 *					arrives.  Typically SET_DESCRIPTOR requests are
 *					not used in most applications, and it is
 *					optional to support this type of request.
 *
 * Note:            None
 *******************************************************************/
void USBCBStdSetDscHandler(void)
{
    // Must claim session ownership if supporting this request
}//end

/*******************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 * 					SET_CONFIGURATION (wValue not = 0) request.  This
 *					callback function should initialize the endpoints
 *					for the device's usage according to the current
 *					configuration.
 *
 * Note:            None
 *******************************************************************/
void USBCBInitEP(void)
{
    CDCInitEP();
}

/*******************************************************************
 * Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *                        USB_EVENT event, void *pdata, WORD size)
 *
 * PreCondition:    None
 *
 * Input:           USB_EVENT event - the type of event
 *                  void *pdata - pointer to the event data
 *                  WORD size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occured.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 *******************************************************************/
BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size)
{
    switch(event)
    {
        case EVENT_CONFIGURED:
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER:
            Nop();
            break;
        default:
            break;
    }
    return TRUE;
}
