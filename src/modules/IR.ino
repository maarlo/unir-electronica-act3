int receiveIrData() {
  if (IrReceiver.decode()) {
    int irData = IrReceiver.decodedIRData.command;
    IrReceiver.resume();

    return irData;
  } else {
    return -1;
  }
}

void processIrData(int irData) {
  switch (irData) {
    case 2:
      batteryTmpOptimal++;
      break;
    case 152:
      batteryTmpOptimal--;
      break;
  }
}