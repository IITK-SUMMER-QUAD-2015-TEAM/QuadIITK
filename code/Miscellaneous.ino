int16_t findMedianIntWithDiff(int16_t *data, int16_t arraySize, int16_t *diff) 
{
  uint16_t temp;
  boolean flag = 0;
  byte i;
  
   // Sorts numbers from lowest to highest
  while (flag != 1) 
  {        
    flag = 1;
    for (i=0; i<(arraySize-1); i++) 
	{
      if (data[i] > data[i+1]) 
	  {     // numbers are out of order - swap
        temp = data[i+1];
        data[i+1] = data[i];
        data[i] = temp;
        flag = 0;
      }
    }
  }

  *diff = abs(data[0] - data[arraySize-1]);
  
  return data[arraySize/2]; // return the median value
}
