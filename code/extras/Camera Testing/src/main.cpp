#include <Arduino.h>
#include <HuskyLens.h>

HUSKYLENS Husky;
HUSKYLENSResult fHusky;
HUSKYLENSResult result;

void printResult();

//1ST
#define MaxWidthOfBlock 100

//2ND
#define MinRatioForBlock 1  // The HEIGH/WIDTH minimum ratio to be a block, anything less than this is a line and the code discards it.



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
      delay(200);
    }   
}

void printResult(){

  int16_t numberOfBlocks = Husky.countBlocksLearned();

  int16_t blocksIndexNumber[numberOfBlocks];
  uint16_t blocksHeight[numberOfBlocks];
  uint16_t blocksWidth[numberOfBlocks];

  int16_t maxHeightIndex = -1;
  int16_t maxHeight = -1;

  for (int i = 0; i < numberOfBlocks; i++)
  {
    blocksIndexNumber[i] = i;
    
    fHusky = Husky.getBlockLearned(i);

    blocksHeight[i] = fHusky.height;
    blocksWidth[i] = fHusky.width;

  }

  //NO CHANGES
  /*
  for (int i = 0; i < numberOfBlocks; i++)
  {
    if((blocksHeight[i] > maxHeight))
    {
      maxHeight = blocksHeight[i];
      maxHeightIndex = blocksIndexNumber[i];
    } 
  }
  */


  //1ST ITINERATION, WE ONLY USE THE MAX. WIDTH OF THE BLOCK
  /*
  for (int i = 0; i < numberOfBlocks; i++)
  {
    if((blocksHeight[i] > maxHeight) && (blocksWidth[i] <= MaxWidthOfBlock))
    {
      maxHeight = blocksHeight[i];
      maxHeightIndex = blocksIndexNumber[i];
    } 
  }
  */

  //2ND ITINERATION, WE USE THE HEIGH/WIDTH RATIO
  for (int i = 0; i < numberOfBlocks; i++)
  {
    if((blocksHeight[i] > maxHeight) && (((double)blocksHeight[i]/(double)blocksWidth[i]) > MinRatioForBlock))
    {
      maxHeight = blocksHeight[i];
      maxHeightIndex = blocksIndexNumber[i];
    }else if (((double)blocksHeight[i]/(double)blocksWidth[i]) > 0.3)
    {
      Serial.println("BARRERA");
      return;
    }
    
  }
  

  //Para evitar errores en casos de que sea necesario
  
  if((maxHeightIndex == -1 ) || (maxHeight == -1)) 
  {
    Serial.println("No hay nada");
  }else{
    fHusky = Husky.getBlockLearned(maxHeightIndex);

    Serial.println(String()+F("Block:xCenter=")+fHusky.xCenter+F(",yCenter=")+fHusky.yCenter+F(",width=")+fHusky.width+F(",height=")+fHusky.height+F(",ID=")+fHusky.ID);
  }
}
