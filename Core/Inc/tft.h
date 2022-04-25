/**
  ******************************************************************************
  * @file    hx8347g.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    06-May-2014
  * @brief   This file contains all the functions prototypes for the hx8347g.c
  *          driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  *   Customizado: 22/06/2020
  *         Autor: Leandro Poloni Dantas
  *   Observações: Este arquivo está compatibilizado no o kit ST NUCLEO-F446RE.
  *  Modificações: - A função reset passa a consider um delay maior antes de
  *  			   escrever em um registrador, segui recomendações do presentes
  *  			   no manual do driver ILI9340.
  *  			   - (01/07/2020)Algumas funções foram criadas para facilitar a
  *  			   integração com uma câmera OV7670.
  *  			   - (16/07/2020) Uma função de inicialização de GPIOs foi
  *  			   incluida para evitar o uso do CubeMX.
  *				   - (16/07/2020) Uma função readID() ganhou uma macro mais
  *  			   intuitiva tft_readID().
  *  			   - (16/07/2020) Correção na macro de acesso a memória
  *  			   (#define pgm_read_word(addr) (*(const unsigned short *)(addr)),
  *  			   antes fazia referência a apenas 16 bits do endereço,
  *  			   agora é capaz de ler toda a memória. Esse falha estava
  *  			   corrompendo o acesso a tabela de fontes LCD quando tinha
  *  			   imagens salvas em flash e o endereço passava de 0xFFFF.
  *  			   - (27/08/2020) Criação de uma nova função para escrita de caracteres com
  *  			   limpeza automática do fundo (write_fillbackground).
  *  			   Criação de novas funções para o usuário com chamada para nova função write:
  *  			   printnewtstr_bg, printstr_bg.
  *  			   Criação da função para troca da cor de fundo do texto setTextBackColor.
  *  			   A documentação de algumas funções foi melhorada ou criada.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HX8347G_H
#define __HX8347G_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "fonts.h"


   
#define true	1
#define false	0




#define MIPI_DCS_REV1   (1<<0)
#define AUTO_READINC    (1<<1)
#define READ_BGR        (1<<2)
#define READ_LOWHIGH    (1<<3)
#define READ_24BITS     (1<<4)
#define XSA_XEA_16BIT   (1<<5)
#define READ_NODUMMY    (1<<6)
#define INVERT_GS       (1<<8)
#define INVERT_SS       (1<<9)
#define MV_AXIS         (1<<10)
#define INVERT_RGB      (1<<11)
#define REV_SCREEN      (1<<12)
#define FLIP_VERT       (1<<13)
#define FLIP_HORIZ      (1<<14)

#define tft_readID()	readID()

void tft_init(uint16_t ID);

void reset(void);

uint16_t readID(void);

void setRotation(uint8_t r);

void invertDisplay(uint8_t i);

void vertScroll(int16_t top, int16_t scrollines, int16_t offset);

void setFont(const GFXfont *f);

void setTextWrap(uint8_t w);

void setTextColor (uint16_t color);

void setTextBackColor (uint16_t color);

void setTextSize (uint8_t size);

void setCursor(int16_t x, int16_t y);

void printnewtstr (int row, uint16_t txtcolor, const GFXfont *f, uint8_t txtsize, uint8_t *str);

void printnewtstr_bc(int row, uint16_t txtcolor, uint16_t txtbackcolor, const GFXfont *f, uint8_t txtsize, uint8_t *str);

void printstr (uint8_t *str);

void printstr_bc (uint8_t *str);

/****************** delay in microseconds ***********************/
void delay (uint32_t time);


/****************** Integração com câmera ***********************/
//Testado com LCF TFT ILI9340
void desenhaPixel(uint16_t pixel);

void inicioDados(void);

void fimDados(void);

/****************** Inicialização de GPIOs **********************/
void tft_gpio_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __HX8347G_H */

