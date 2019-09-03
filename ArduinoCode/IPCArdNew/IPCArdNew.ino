void setup()
{
  Serial.begin(115200); //serial begin
}

void loop()
{
  String readStr = ""; //some variables
  String readVal = "";
  double inVal, outVal;
  
  if (Serial.available()){ //when serial data comes from modelica
  while(Serial.available()){
    char readChar = (char)Serial.read();
    readStr+=readChar; 
    if(readChar == '\n') break;
  } //read the data and store in a string
    for (int i = 1; i < (readStr.length()-1); i++)
    {
      readVal += readStr[i];
    }
    inVal = readVal.toDouble(); //extract value
    outVal = inVal/2;
    Serial.print("1," + String(outVal) + "\n"); //send data in same format i.e. ending with \n character
    delay(1);
  } 
}
