//
//  EK304SD.cpp
//  EK304SD
//
//  Created by Renan Cintra Villaca on 14/11/19.
//  Copyright © 2019 UTFPR. All rights reserved.
//

#include "EK304SD.h"

void writeData(String fileNameString, String data){
  char fileName[10];
  fileNameString.toCharArray(fileName, sizeof(fileName));
  
  // Inicializa o modulo SD
  //if(!sdCard.begin(chipSelect,SPI_HALF_SPEED))sdCard.initErrorHalt();
  
  // Abre o arquivo fileName
  if (!meuArquivo.open(fileName, O_RDWR | O_CREAT | O_AT_END))
  {
    sdCard.errorHalt("Erro na abertura do arquivo!");
  }
  
  //Grava os dados no SD Card
  if (data != NULL){
    meuArquivo.println(data);
    }
    else{
      meuArquivo.println("");
    }

  meuArquivo.close(); //fecha o arquivo
  
}


void readData(String fileNameString){
  char fileName[10];
  fileNameString.toCharArray(fileName, sizeof(fileName));
  
  // Inicializa o modulo SD
  //if(!sdCard.begin(chipSelect,SPI_HALF_SPEED))sdCard.initErrorHalt();
  
  // Abre o arquivo fileName
  if (!meuArquivo.open(fileName, O_READ))
  {
    sdCard.errorHalt("Erro na abertura do arquivo!");
  }

  Serial.write(meuArquivo.read());

  meuArquivo.close();
  
  }


void clearData(String fileNameString){
  char fileName[10];
  fileNameString.toCharArray(fileName, sizeof(fileName));
  
  // Inicializa o modulo SD
  //if(!sdCard.begin(chipSelect,SPI_HALF_SPEED))sdCard.initErrorHalt();
  
  // Abre o arquivo fileName
  if (!meuArquivo.open(fileName, O_RDWR))
  {
    sdCard.errorHalt("Erro na abertura do arquivo!");
  }

  if (!meuArquivo.remove()){
      Serial.println("Erro na remoCao do arquivo");
     }
  }

void ini(){
    if(!sdCard.begin(chipSelect,SPI_HALF_SPEED))sdCard.initErrorHalt();
}
