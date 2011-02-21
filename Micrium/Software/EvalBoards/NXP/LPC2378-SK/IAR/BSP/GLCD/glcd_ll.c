/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    (c) Copyright IAR Systems 2006
 *
 *    File name   : glcd_ll.c
 *    Description : GLCD low level functions
 *
 *    History :
 *    1. Date        : December 2, 2006
 *       Author      : Stanimir Bonev
 *       Description : Create
 *
 *    $Revision: 1.0 $
 **************************************************************************/
#include <iolpc2378.h>
#include <board.h>
#include <glcd_ll.h>

/*************************************************************************
 * Function Name: LcdSetReset
 * Parameters: Boolean State
 * Return: none
 *
 * Description: Set reset pin state
 *
 *************************************************************************/
void LcdSetReset (Boolean State)
{
	if(State)
	{
    LCD_RST_FSET = LCD_RST_MASK;
	}
	else
	{
    LCD_RST_FCLR = LCD_RST_MASK;
	}
}

/*************************************************************************
 * Function Name: LcdLLInit
 * Parameters: none
 * Return: none
 *
 * Description: Init Reset and Backlight control outputs
 *
 *************************************************************************/
void LcdLLInit (void)
{
	// LCD Reset output
    LCD_RST_FDIR |= LCD_RST_MASK;

  LcdSetReset(0);
}

/*************************************************************************
 * Function Name: Ssp0ChipSelect
 * Parameters: Boolean Select
 * Return: none
 *
 * Description: SSP0 Chip select control
 * Select = true  - Chip is enable
 * Select = false - Chip is disable
 *
 *************************************************************************/
void Ssp0ChipSelect (Boolean Select)
{
  if (Select)
  {
    LCD_CS_FCLR = LCD_CS_MASK;
  }
  else
  {
    LCD_CS_FSET = LCD_CS_MASK;
  }
}

/*************************************************************************
 * Function Name: Ssp0SetWordWidth
 * Parameters: Int32U Width
 * Return: Boolean
 *
 * Description: Set SSP 0 word width
 *
 *************************************************************************/
Boolean Ssp0SetWordWidth (Int32U Width)
{
  if(4 > Width || Width > 16)
  {
    return(FALSE);
  }

  SSP0CR0_bit.DSS = Width - 1;
  return(TRUE);
}

/*************************************************************************
 * Function Name: Ssp0SetClockFreq
 * Parameters: Int32U Frequency
 * Return: Int32U
 *
 * Description: Set SSP 0 clock
 *
 *************************************************************************/
Int32U Ssp0SetClockFreq (Int32U Frequency)
{
Int32U Fspi = SYS_GetFpclk(SSP0_PCLK_OFFSET);
Int32U Div = 2;
  while((Div * Frequency) < Fspi)
  {
    if((Div += 2) == 254)
    {
      break;
    }
  }
  SSP0CPSR = Div;
  return(Fspi/Div);
}

/*************************************************************************
 * Function Name: Ssp0Init
 * Parameters: Int32U Clk, Int32U Width
 * Return: none
 *
 * Description: Init SSP0
 *
 *************************************************************************/
void Ssp0Init(Int32U Clk, Int32U Width)
{
  // Assign GPIO to SSP0 - SCK, MOSI, MISO
  PINSEL3_bit.P1_20 = 3;  // SCK
  PINSEL3_bit.P1_23 = 3;  // MISO
  PINSEL3_bit.P1_24 = 3;  // MOS1

  // Chip select
  LCD_CS_FDIR |= LCD_CS_MASK;

  Ssp0ChipSelect(FALSE);

  // Spi init
  PCONP_bit.PCSSP0 = 1;			// SSP0 clock enable
  SSP0CR1_bit.SSE  = 0;     // Disable module
  SSP0CR1_bit.LBM  = 0;     // Disable Loop Back Mode
  SSP0CR1_bit.MS   = 0;     // Master mode
  SSP0CR0_bit.FRF  = 0;			// SPI
  SSP0CR0_bit.SPO  = 0;
	SSP0CR0_bit.SPH  = 0;
  SSP0IMSC = 0;             // disable all interrupts
  SSP0DMACR = 0;            // disable DMA
  SSP0CR1_bit.SSE  = 1;     // Enable module
  for (Int32U i = 0; i < 8; i++ )
  {
    volatile Int32U Dummy = SSP0DR; // clear the RxFIFO
  }
  // Set SSP clock frequency
  Ssp0SetClockFreq(Clk);
  // Set data width
  Ssp0SetWordWidth(Width);

}

/*************************************************************************
 * Function Name: Ssp0TranserByte
 * Parameters: Int32U Data
 * Return: Int32U
 *
 * Description: Transfer byte from SSP0
 *
 *************************************************************************/
Int32U Ssp0TranserByte (Int32U Data)
{
  while(!SSP0SR_bit.TNF);
  SSP0DR = Data;
  while(SSP0SR_bit.BSY);
  return(SSP0DR);
}

/*************************************************************************
 * Function Name: Ssp0SendBlock
 * Parameters: pInt8U pData, Int32U Size
 *
 * Return: void
 *
 * Description: Write block of data to SSP
 *
 *************************************************************************/
void Ssp0SendBlock (pInt8U pData, Int32U Size)
{
Int32U OutCount = Size;
  while (OutCount)
  {
    while(SSP0SR_bit.TNF && OutCount)
    {
      SSP0DR = *pData++ | 0x100;  // Data
      --OutCount;
    }
  }
  while(SSP0SR_bit.BSY);
  // draining RX Fifo
  while (SSP0SR_bit.RNE)
  {
    volatile Int32U Dummy = SSP0DR;
  }
}

/*************************************************************************
 * Function Name: Ssp0ReceiveBlock
 * Parameters: pInt8U pData, Int32U Size
 *
 * Return: void
 *
 * Description: Read block of data from SSP
 *
 *************************************************************************/
void Ssp0ReceiveBlock (pInt8U pData, Int32U Size)
{
Int32U Delta = 0;
  while (Size || Delta)
  {
    while(SSP0SR_bit.TNF && (Delta < SSP_FIFO_SIZE) && Size)
    {
      SSP0DR = 0xFFFF;
      --Size; ++Delta;
    }
    while (SSP0SR_bit.RNE)
    {
      *pData++ = SSP0DR;
       --Delta;
    }
  }
}
