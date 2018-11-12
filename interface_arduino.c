#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h> //para leitura do sensor DHT

#define F_CPU 16000000UL // 16 MHz, clock_ms externo, define necessario para usar delay corretamente
#define BAUDRATE 9600        //taxa de transmissao em bps
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)

// Pinos associados aos sensores
#define DHT_PIN PC0
#define THERMISTOR PC1   //A1

#define THER_REF 27/510.0
/******************** Funcoes **********************/
//Leitura dos sensores
uint8_t readDHT_byte();//funcao auxiliar de readDHT(data[])
bool readDHT(uint8_t data[]); //Funcao que le do sensor DHT os dados de umidade e temperatura, data[0:1]:umidade, data[1:2]:Temperatura
uint8_t DHT_readUmidade(); //retorna data[0] direto(se utiliza das funcoes acima)
//faz a leitura do sensor de temperatura associado ao pino THERMISTOR, retorna o valor em graus Celcius
//mapeados da seguinte forma(0:54 <=> 0:1023)
uint8_t readTherm();

//Comunicacao USART
void USART_initialize();
void USART_transmision(uint8_t data);
void USART_transmisionString(uint8_t *data);
uint8_t USART_reception();
void USART_flush();

//Comunicacao SPI
void SPI_MasterInit(void);
uint8_t SPI_MasterTransmit(uint8_t cData);

void setup()
{
  // Pre-config. para leitura do sensor DHT
  DDRC |= 1 << DHT_PIN;
  PORTC|= !(1 << DHT_PIN);      //coloca DHT_PIN para nivel alto

  USART_initialize();
  SPI_MasterInit();

  sei();
}

volatile bool enable = false;
volatile uint8_t pwm;

int main()
{
  uint8_t temp;
  uint8_t umid;
  while(true)
  {
    if(enable)
    {
      umid = readDHT_byte();
      temp = readTherm();

      pwm = SPI_MasterTransmit(temp);
      SPI_MasterTransmit(umid);
    }
  }

  return 0;
}
/******************INTERRUPCOES******************/
enum COMMAND{
  ON  = 0x01,
  OFF = 0x02,
  REQ = 0x03,
};

ISR(USART_RX_vect){
  uint8_t data = USART_reception();
  USART_flush();

  switch (data) {
    case ON:
      // ligar o sistema
      enable = true;
    break;
    case OFF:
      //desligar o sistem
      enable = false;
    break;
    case REQ:
      // RETORNA utimo PWM(%)
      USART_transmision(pwm);
    break;
    default://comando nao reconhecido
      //ignorar
     break;
  }
}

/****************************IMPLEMENTACOES**********************/
//Leitura dos sensores
uint8_t DHT_readUmidade(){
  uint8_t data[5];
  readDHT(data);
  return data[0];
}
bool readDHT(uint8_t data[]){
  uint8_t dht_in = 0;
  //Pedido de transmissao de dados
  DDRC |= 1 << DHT_PIN;
  PORTC&= !(1<<DHT_PIN);
  _delay_ms(18); //ate 18ms
  PORTC|= (1<<DHT_PIN);
  _delay_us(1);

  //Confirmacao do DHT
  DDRC &= !(1<<DHT_PIN);
  _delay_us(80); //ate 80us
  dht_in = PINC & (1<<DHT_PIN);
  if(dht_in)
    return false;

  _delay_us(80);//ate 80us
  dht_in = PINC & (1<<DHT_PIN);
  if(!dht_in)
    return false;

  //inicio da transmissao dos dados
  _delay_us(80);
  for(uint8_t i = 0; i < 5; i++){
    data[i] = readDHT_byte();
  }

  uint8_t checksum = data[0] + data[2];

  if(checksum != data[4])
    return false;

  return true;
}
uint8_t readDHT_byte(){
  uint8_t data = 0;
  for(uint8_t i = 0; i < 8; i++){
    while( !(PINC & (1<<DHT_PIN)));//espera sair da zona LOW
    _delay_us(30);
    if( PINC & (1<<DHT_PIN) )//se ler nivel Alto nessa linha, entao o bit eh 1
      data |= 1 << (7-i);
    while( PINC & (1<<DHT_PIN) );//espera receber o nivel LOW indicando o proximo bit
  }
  return data;
}
//Realiza a conversao ADC e retorna o valor em Graus Celcius
uint8_t readTherm(){
  ADCSRA |= 0b10000111; //divide o clock_ms por 128 (o clock_ms de conversao sera 16Mhz/128)

  ADMUX  |= 0b01000000;//usa o Vcc como ref
  ADMUX  |= 0b00000001; //converte do pino A1
  ADCSRA |= 0b01000000; // inicia a conversao A/D
  while(!(ADCSRA & 0b00010000)); //Aguarda a conversao ser concluida
  return (int)(ADC*THER_REF) >> 2; //desloca(ignorando os dois bits menos signicativos) para ADC ocupar um byte
}

//Comunicacao SPI
void SPI_MasterInit(void)
{
  /* Set MOSI and SCK output, all others input */
  DDRB = (1<<DDB3)|(1<<DDB5);
  /* Enable SPI, Master, set clock rate fck/4 */
  SPCR = (1<<SPE)|(1<<MSTR);
}
uint8_t SPI_MasterTransmit(uint8_t cData)
{
  /* Start transmission */
  SPDR = cData;
  /* Wait for transmission complete */
  while(!(SPSR & (1<<SPIF)));
  return SPDR;
}
// Comunicacao USART
void USART_initialize(){
  //habilita a comunicacao USART
  //habilita entrada (Rx) e saida(Tx)
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
  UCSR0C = (3<<UCSZ00);

  // definir a taxa de transmissao
  UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8);
  UBRR0L = (uint8_t)(BAUD_PRESCALLER);
  //formato do frame 8bits de dado e 1stop bits
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);//ativa o receptor e o transmissor
  UCSR0C = (3<<UCSZ00);
  //Habilita interrupcoes locais
  UCSR0B |= (1 << RXCIE0) | (1 << TXCIE0);
}
void USART_transmisionString(uint8_t *data){
  while(*data != 0x00){
    USART_transmision(*data);
    data++;
  }
}
//Coloca um byte no buffer de saida de dado
void USART_transmision(uint8_t data){
  //Aguarda a ultima transmissao ser concluida
  while( !(UCSR0A & (1<<UDRE0)));
  //coloca o byte no buffer
  UDR0 = data;
}
//Aguarda a recepcao ser concluida
uint8_t USART_reception(){
  //aguardar buffer a finalizacao da recepcao
  while ( !(UCSR0A & (1 << RXC0)) );
  return UDR0;
}
//Limpa o buffer de entrada de dados
void USART_flush(){
  uint8_t trash;
  while ( (UCSR0A & (1<<RXC0))) trash = UDR0;
}
