#include <Wire.h>
//#include "L3G.h"
#include <TinyGPS++.h>
#include <Adafruit_MPL115A2.h>
#include <SoftwareSerial.h>
#include "RTClib.h"
#include <SFE_BMP180.h>
#include <LSM303.h>
#include "i2c.h"
#include "i2c_L3G.h"
#include <SD.h>
#include <SPI.h>
#include <EEPROM.h>


double Altitude, preAltitude;
double Pressure;
double baseline;
int pinCS = 53;

//String gpgga;
//int dataBoyutu = 0;

byte uyduDusuyorMu;
byte kilitAcikMi;

int address = 0;
int paketSayisi;

const int motorPin1 = 3;
const int motorPin2 = 5;
const int motorPin3 = 7;
const int motorPin4 = 9;
int bekleme = 2; //mili saniye

int ilkDerece;

float pilSeviyesi;

typedef struct
{
  const int32_t takimno = 43420;
  int paketNumarasi;
  String gondermeZamani;
  float basinc;
  float yukseklik;
  float inisHizi;
  float sicaklik;
  int pilGerilimi;
  float GPS_Latitude;
  float GPS_Longitude;
  float GPS_Altitude;
  int uydu_statusu;
  float pitch;
  float roll;
  float yaw;
  int donus_sayisi;
  //float derecePusula;

} tdata;

tdata datas;
SoftwareSerial gpsSerial(11, 13); // 11 -> RX, 13 -> TX
RTC_DS3231 rtc;
//char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
Adafruit_MPL115A2 mpl115a2;
TinyGPSPlus gps; // create gps object
L3G l3g;
SFE_BMP180 pressure;

LSM303 compass;

void setup() {
  //Seri port baslangiclari
  Wire.begin();
  Serial.begin(57600);
  gpsSerial.begin(57600);

  datas.paketNumarasi = 0 ;


  rtc.begin();
  /*
    if (! rtc.begin()) {
    while (1);
    }*/

  /*
    if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");

    // Comment out below lines once you set the date & time.
    // Following line sets the RTC to the date & time this sketch was compiled
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

    // Following line sets the RTC with an explicit date & time
    // for example to set January 27 2017 at 12:56 you would call:
    //rtc.adjust(DateTime(2019, 06, 26, 13, 27, 00));
    }*/
  //rtc.lostPower();

  mpl115a2.begin();
  /*
    gyro.init();
    gyro.enableDefault();*/

  pressure.begin();

  // get baseline pressure
  //baseline = getPressure();
  for (int i = 0; i < 10; i++) {
    baseline += getPressure();
  }
  baseline /= 10;

  //-------------LSM303------------
  compass.init();
  compass.enableDefault();

  compass.m_min = (LSM303::vector<int16_t>) {
    -32767, -32767, -32767
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    +32767, +32767, +32767
  };

  compass.read();
  for (int i = 0; i < 10; i++) {
    ilkDerece += compass.heading((LSM303::vector<int>) {
      0, 0, 1
    });
  }

  ilkDerece /= 10;
  //------------/LSM303---------------------;

  //---------------L3G----------------------
  l3g.initialize();
  //-------------/L3G---------------------;

  pinMode(pinCS, OUTPUT);
  SD.begin();

  //Create/Open file
  File myFile = SD.open("deneme.csv", FILE_WRITE);

  //if the file opened okay, write to it:
  if (myFile) {
    //Write to file
    //String veri = "takim_no, paket_numarasi, gonderme_zamani, basinc, yukseklik, inis_hizi, sicaklik, pil_gerilimi, gps_latitude, gps_longtitude, gps_altitude, uydu_statusu, pitch, roll, yaw"; //pusula silindi
    if (!SD.exists("girdi.txt")) {
      myFile.println("takim_no,paket_numarasi,gonderme_zamani,basinc,yukseklik,inis_hizi,sicaklik,pil_gerilimi,gps_latitude,gps_longtitude,gps_altitude,uydu_statusu,pitch,roll,yaw,donus_sayisi");
      myFile.close();
      File dosya = SD.open("girdi.txt", FILE_WRITE);
      dosya.close();
    }

  }
  //------------------Servo pinlerini tanıt--------------------------
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  //-------------------Servo----------------------------------------

  //--------------EEPROM-------------------------
  //EEPROM en başta bir kere tüm adresleri sıfırlanmalı sonra çalıştırılmalıdır
  /*
    for(int i = 0; i < 4096; i++){
        EEPROM.put(i, 0);
    }

  */

  paketSayisi = EEPROM.get(address, paketSayisi);

  //-------------EEPROM-------------------------------

  //-----------------Buzzer---------------
  pinMode(19, OUTPUT);
  //------------------/Buzzer---------------;

  GPSEksenler();
}

byte hareketEttiMi() {
  float xyz_dps[3];
  l3g.getMeasurement(xyz_dps);

  if (10 < xyz_dps[2] || xyz_dps[2] < -10) {
    return 1;
  } else {
    return 0;
  }

}



String floatFormat(float sayi, int stringLenght, int noktaSonrasiSayi) {
  char outstr[50];
  dtostrf(sayi, stringLenght, noktaSonrasiSayi, outstr);
  String a = outstr;
  return outstr;
}

void kilit(int adimSayisi, byte ac = 1) { // ac -> 1 ise açılır, ac -> 0 ise kapatılır.
  //float adim = 512.0 * turSayisi;
  if (ac == 1 && kilitAcikMi == 0) {
    for (int i = 0; i < adimSayisi; i++)
    {
      adim1();
      adim2();
      adim3();
      adim4();
    }
    kilitAcikMi = 1;
  } else if (ac == 0 && kilitAcikMi == 1) {
    for (int i = 0; i < adimSayisi; i++)
    {
      adim4();
      adim3();
      adim2();
      adim1();
    }
    kilitAcikMi = 0;
  }

}


void gyroEksenler() {

  compass.read();
  float yaw = 0;
  for (int i = 0; i < 10; i++) {
    yaw += compass.heading((LSM303::vector<int>) {
      0, 0, 1
    }); // x yaw
  }
  yaw /= 10;
  float pitch = 0;
  for (int i = 0; i < 10; i++) {
    pitch += compass.heading((LSM303::vector<int>) {
      1, 0, 0
    }); // y pitch
  }
  pitch /= 10;
  float roll = 0;
  for (int i = 0; i < 10; i++) {
    roll += compass.heading((LSM303::vector<int>) {
      0, 1, 0
    }); // z roll
  }
  roll /= 10;

  donusSayisi(roll); // donduyse değeri bir artırır

  datas.pitch = pitch;
  datas.yaw = yaw;
  datas.roll = roll;
}

void GPSEksenler() {
  float latitude, longitude, altitude;
  //Altitude = 0;
  while (true) {
    while (gpsSerial.available() > 0) {
      char data = gpsSerial.read();
      if (gps.encode(data)) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        Altitude = gps.altitude.meters();
        datas.GPS_Latitude = latitude;
        datas.GPS_Longitude = longitude;
        datas.GPS_Altitude = altitude;
        if (latitude != 0 && longitude != 0 && altitude > 0) {
          break;
        } else if (latitude == 0 && longitude == 0 && altitude == 0) {
          break;
        }
        /*buyuk eşittiri sonradan kaldırman gerek açık alanda çalışması için
          else if (latitude == 0 && longitude == 0 && Altitude == 0) {
          break;
          }*/

      }
    }
    if (latitude != 0 && longitude != 0 && altitude > 0) {
      break;
    } else if (latitude == 0 && longitude == 0 && altitude == 0) {
      break;
    }
    /*
      else if (latitude == 0 && longitude == 0 && Altitude == 0) {
      break;
      }*/
  }
}


void volt() {
  int value = analogRead(0);
  float vin = (value / 5.0) * 1023.0; // calculate the ratio
  datas.pilGerilimi = vin;
  //pilSeviyesi = (vin * 100) / 4.37; // 4.37 pilin max vermesi beklenen volt, min 2.8 volt ve bu değeri 0 kabul ederek yeni bir hesap yap
}

void yukseklik() {
  Pressure = 0;
  for (int i = 0; i < 10; i++) {
    Pressure += getPressure();
  }

  Pressure /= 10;

  Altitude = 0;
  for (int i = 0; i < 10; i++) {
    Altitude  += pressure.altitude(Pressure, baseline);

  }

  Altitude /= 10;

  if (Altitude <= 0) {
    Altitude = 0;
  }

  datas.yukseklik = Altitude;
}

void temperature() {
  float temperatureC = 0;
  for (int i = 0; i < 10; i++) {
    temperatureC += mpl115a2.getTemperature();
  }

  temperatureC /= 10;

  datas.sicaklik = temperatureC;

}

void pressureKPa() {
  float pressureKPA;
  //pressureKPA = mpl115a2.getPressure();
  pressureKPA = Pressure / 10; // kPa
  datas.basinc = pressureKPA;
}

void buzzerAc(byte ac = 0) {
  if (ac == 1) {
    digitalWrite(19, HIGH);
  } else if (ac == 0) {
    digitalWrite(19, LOW);
  }
}

void saat() {
  DateTime now = rtc.now();
  String saatModul = "";
  saatModul += now.day();
  saatModul += '/';
  saatModul += now.month();
  saatModul += '/';
  saatModul += now.year();
  saatModul += '-';
  saatModul += now.hour();
  saatModul += ':';
  saatModul += now.minute();
  saatModul += ':';
  saatModul += now.second();
  datas.gondermeZamani = saatModul;

}

void uyduStatusu() { ///////////////////BİTİR/////////////////
  /*  0 -> duruyor => gyrodan veya yukseklikten yapılabilir
      1 -> çıkıyor => gyro veya yukseklik ama gyro daha iyi olur
      2 -> en tepede => gyro veya yukseklil ama gyro daha iyi olur
      3 -> iniyor
      4 -> kilit mekaniz ması açıldı => servo çalışınca sinyal gonder
      5 -> indi-kurtarılmayı bekliyor => yukseklik veya gyro

  */
  if (uyduDusuyorMu == 0) {
    if (Altitude <= 0 || Altitude < 1) {
      datas.uydu_statusu = 0;
    } else if (Altitude >= 1) {
      datas.uydu_statusu = 1;
    } else if (Altitude >= 498) {
      datas.uydu_statusu = 2;
      uyduDusuyorMu = 1; // bu aslında uydununun bu aşamadan sonra inmeye geçeceğini kontrol eder, değişken adı yanıltmasın, bu değişken kilitde kontrol amaçlı kullanılıyor
    }
  } else {
    if (Altitude <= 500) {
      datas.uydu_statusu = 3;
    } else if (Altitude <= 405) {
      kilit(230);
      buzzerAc(1);
      datas.uydu_statusu = 4;
    } else if (Altitude <= 0 || Altitude < 1) {
      datas.uydu_statusu = 5;
    }
  }


}

void donusSayisi(float derece) {//gyro eksenler içinde kullanıldı
  int dereceInt = (int) derece;

  if (hareketEttiMi() == 1) {
    if ((0 < ilkDerece - dereceInt && ilkDerece - dereceInt < 10) || (0 < dereceInt - ilkDerece && dereceInt - ilkDerece < 10)) {
      datas.donus_sayisi++;
    }
  }

}

void gonderYazdir(byte hasExtraData = 0) {
  paketSayisi++;
  EEPROM.put(address, paketSayisi);

  datas.paketNumarasi = paketSayisi;
  String data = "";
  data += datas.takimno;
  data += ',';
  data += datas.paketNumarasi;
  data += ',';
  data += datas.gondermeZamani;
  data += ',';
  data += datas.basinc;
  data += ',';
  data += datas.yukseklik;
  data += ',';
  data += datas.inisHizi;
  data += ',';
  data += datas.sicaklik;
  data += ',';
  data += datas.pilGerilimi;
  data += ',';
  data += floatFormat(datas.GPS_Latitude, 9, 6);//noktadan sonraki 6 basamagi da gorunmesi icin formatladim
  data += ',';
  data += floatFormat(datas.GPS_Longitude, 9, 6);
  data += ',';
  data += datas.GPS_Altitude;
  data += ',';
  data += datas.uydu_statusu;
  data += ',';
  data += datas.pitch;
  data += ',';
  data += datas.roll;
  data += ',';
  data += datas.yaw;
  data += ',';
  data += datas.donus_sayisi;
  //data += datas.derecePusula;

  File myFile = SD.open("data.csv", FILE_WRITE);

  if (myFile) {
    myFile.println(data);
    myFile.close();
  }

  data += ',';
  //data += pilSeviyesi;
  //data += ',';
  if (hasExtraData == 1) {
    data += getSifre();
    data += ',';
  }


  Serial.println(data);


}

double hiz() {
  int inisHizi = Altitude  - preAltitude;
  preAltitude = Altitude;
  if (inisHizi < 0) {
    return 0;
  } else {
    return inisHizi;
  }

}

double getPressure()
{
  char status;
  double T, P, p0, a;

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    status = pressure.getTemperature(T);
    if (status != 0)
    {

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          return (P);
        }

      }

    }

  }

}

/*
  void pusula() {
  compass.read();
  float heading = compass.heading();
  datas.derecePusula = heading;
  }
*/
void adim1() {
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
  delay(bekleme);
}

void adim2() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
  delay(bekleme);
}

void adim3() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);
  delay(bekleme);
}

void adim4() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, HIGH);
  delay(bekleme);
}

String sifreAyir(String sifre) {
  String a;

  int adim = 7;
  while (1) {
    if (sifre[adim] == '\n') {
      break;
    }
    a += sifre[adim];
    adim++;
  }

  return a;
}

byte isSifre(String sifre) {
  byte sifreMi = 0;
  if (sifre[0] == '<' && sifre[1] == 's' && sifre[2] == 'i' && sifre[3] == 'f' && sifre[4] == 'r' && sifre[5] == 'e' && sifre[6] == '>' && sifre[7] != '\n') {
    sifreMi = 1;
  }
  return sifreMi;
}

void sdSifreKaydet(String dosyaAdi, String sifre) {
  File myFile = SD.open(dosyaAdi, FILE_WRITE);

  if (myFile) {
    myFile.print(sifre);
    myFile.close();
  }
}

String getSifre() {
  File myFile = SD.open("sifre.txt");
  String sifre;
  if (myFile) {
    while (myFile.available()) {
      sifre = myFile.read();
    }
    myFile.close();
  }

  return sifre;
}

void EEPROMSifirla() {
  for (int i = 0; i < 4096; i++) {
    EEPROM.put(i, 0);
  }
}

void loop() {

  if (Serial.available() > 0) {
    String gelenData = Serial.readString();
    if (gelenData == "kilit_ac\n") {
      kilit(230, 1);
    } else if (gelenData == "kilit_kapa\n") {
      kilit(230, 0);
    } else if (gelenData == "buzzer_ac\n") {
      buzzerAc(1);
    } else if (gelenData == "buzzer_kapa\n") {
      buzzerAc(0);
    } else if (isSifre(gelenData)) {
      sdSifreKaydet("sifre.txt", sifreAyir(gelenData));
    } else if (gelenData == "sifre_kontrol\n") {
      gonderYazdir(1);
    } else if (gelenData == "eeprom_sifirla\n") {
      EEPROMSifirla();
    }
  }

  saat();
  yukseklik(); // içinde basınc heasbı olduğu için ilk değerin de yanlış gelmemesi için önce yukseklik hesaplanmalı
  pressureKPa();
  hiz();
  temperature();
  volt();
  GPSEksenler();
  gyroEksenler();
  uyduStatusu();
  //donusSayisi();
  //pusula(); // z eksenindeki donme açısıdır yani roll değerinden de ulaşılabilir!!!
  gonderYazdir();

  delay(1000);

}
