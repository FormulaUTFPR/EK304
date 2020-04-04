//
//  EK304SD.h
//  EK304SD
//
//  Created by Renan Cintra Villaca on 14/11/19.
//  Copyright © 2019 UTFPR. All rights reserved.
//

#ifndef EK304SD_h
#define EK304SD_h

#include <SdFat.h>

SdFat sdCard;
SdFile meuArquivo;

void ini(); //inicia o cartao de memoria, chamar apenas uma vez no setup
void writeData(String fileNameString, String data);
void readData(String fileNameString);
void clearData(String fileNameString);

#endif /* EK304SD_h */
