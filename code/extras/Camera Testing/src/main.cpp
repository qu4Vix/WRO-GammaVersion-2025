#include <Arduino.h>
#include <HuskyLens.h>

HUSKYLENS Husky;
HUSKYLENSResult fHusky;
HUSKYLENSResult result;

void printResult();



void setup() {
  Serial.begin(115200);
  Serial.println("Programa iniciado");

  Wire.begin(18,19);
  while(!Husky.begin(Wire)) delay(100);

  if(Husky.begin(Wire)==1) Serial.println("Conectada la Husky");

  Husky.writeAlgorithm(ALGORITHM_COLOR_RECOGNITION);

}

void loop() {
  while (Husky.requestBlocksLearned()) 
    {
      printResult();
      delay(100);
    }   
}

void printResult(){

  int16_t numberOfBlocks = Husky.countBlocksLearned();

  int16_t blocksIndexNumber[numberOfBlocks];
  uint16_t blocksHeight[numberOfBlocks];

  int16_t maxHeightIndex;
  uint16_t maxHeight = 0;

  for (int i = 0; i < numberOfBlocks; i++)
  {
    blocksIndexNumber[i] = i;
    
    fHusky = Husky.getBlockLearned(i);
    blocksHeight[i] = fHusky.height;
  }

  for (int i = 0; i < numberOfBlocks; i++)
  {
    if(blocksHeight[i] > maxHeight)
    {
      maxHeight = blocksHeight[i];
      maxHeightIndex = blocksIndexNumber[i];
    } 
  }

  fHusky = Husky.getBlockLearned(maxHeightIndex);

  Serial.println(String()+F("Block:xCenter=")+fHusky.xCenter+F(",yCenter=")+fHusky.yCenter+F(",width=")+fHusky.width+F(",height=")+fHusky.height+F(",ID=")+fHusky.ID);
}
