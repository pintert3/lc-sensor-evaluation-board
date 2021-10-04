
const unsigned int FILE_LINE_LENGTH = 512;

void setup() {
  Serial.begin(9600);
  delay(8000);
  String testData[6] = {String("boys"), String("valedictorians"), String("ant"), String("carpets"), String("smile"), String("sugars") };
  for (int i = 0; i < 6; i++) {
    formatData(testData[i]);
  }
}

void loop() {
}

void formatData(String input) {
  // formats input char string to fixed length output with \t at end padded
  // with zero characters ('0')

  int original_length = input.length();
  Serial.print("length is ");
  Serial.println(original_length);
  if (original_length < FILE_LINE_LENGTH) {
    int numzeros = FILE_LINE_LENGTH - original_length - 1;
    input += '\t';
    for (int i = 0; i < numzeros; i++) {
      input += '0';
    }
  }
  Serial.println(input+String("."));
  Serial.println(input.length());
}
