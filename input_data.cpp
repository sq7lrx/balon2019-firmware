String ident = "identyfikator_nasz";
int g=18, m=20, s=15, wysokosc=15, cisnienie=1023,  co2=40, ozon=20,uv=50, pyl=10;
float dlugosc=19.28, szerokosc=51.47, temperatura=21.8, wilgotnosc=50;
char dlugosc_kier='E', szerokosc_kier='N';
String output="";


void input_data()
{
    output=ident+","+String(g)+","+String(m)+","+String(s)+","+String(dlugosc)+String(dlugosc_kier)+","+String(szerokosc)+String(szerokosc_kier)+","+String(wysokosc)+","+String(temperatura)+","+String(cisnienie)+","+String(wilgotnosc)+","+String(co2)+","+String(ozon)+","+String(uv)+","+String(pyl);
  Serial.println(output);
}
