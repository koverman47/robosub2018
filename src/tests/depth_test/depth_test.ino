int depthPin = 0; 

//float surfacePSI = 10.44; // Check ad hoc
float surfacePSI = -1;
float psi;
float depth = 0.0;
float analog;

void setSurfacePSI() {
	if(surfacePSI == -1) {
		sensePSI();
		surfacePSI = psi;
	}
}


void sensePSI() {
	analog = analogRead(depthPin);
	psi = (analog * 0.0048828125 - 1) * 12.5;
}


void calcDepthUpdate() {
	sensePSI();
	depth = ((analog * 0.0048828125 - 1) * 12.5 - surfacePSI) * 0.13197839577;
}

void setup() {
	Serial.begin(9600);
	setSurfacePSI();
	calcDepthUpdate();

}

int counter = 0;
void loop() {
	//calcDepthUpdate();
	//Serial.print("Analog: ");
	//Serial.println(analog);

	//Serial.print("PSI: ");
	//Serial.println(psi);

	//Serial.print("Depth: ");
	//Serial.println(depth);
  counter++;
  
}
