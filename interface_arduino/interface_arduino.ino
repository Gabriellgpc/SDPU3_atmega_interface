#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h> //para leitura do sensor DHT

#define F_CPU 16000000UL // 16 MHz, clock_ms externo, define necessario para usar delay corretamente
#define BAUDRATE 9600        //taxa de transmissao em bps
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)

// Pinos associados aos sensores
#define THERMISTOR PC1   //A1
#define LUMI_PIN PC0

#define LOAD_PIN PD7

/******************** Funcoes **********************/
//Leitura dos sensores

//faz a leitura do sensor de temperatura associado ao pino THERMISTOR, retorna o valor em graus Celcius
//mapeados da seguinte forma(0:54 <=> 0:1023)
uint8_t readTherm();
uint8_t readLumin();
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
  DDRD |= (1 << LOAD_PIN);
  PORTD&= ~(1 << LOAD_PIN);

  USART_initialize();
  SPI_MasterInit();
}


int main()
{
  uint8_t temp;
  uint8_t lumin;
  uint8_t pwm;

  setup();
  while(true)
  {
     temp = readTherm();
     lumin= readLumin();

     pwm = SPI_MasterTransmit(temp); //manda temp e recebe o ultimo PWM calculado
     SPI_MasterTransmit(lumin);//envia informacoes o ADC relativo a leitura ADC do sensor de luminosidade
     pwm = (pwm >> 1) | ((pwm & 0x01) << 7);

     USART_transmision(pwm);//repassa a informacao do pwm via serial USB da placa arduino
  }

  return 0;
}

/****************************IMPLEMENTACOES**********************/
//Realiza a conversao ADC e retorna o valor em Graus Celcius
uint8_t readTherm(){
  ADCSRA |= 0b10000111; //divide o clock_ms por 128 (o clock_ms de conversao sera 16Mhz/128)

  ADMUX  |= 0b01000000;//usa o Vcc como ref
  ADMUX  |= 0b00000001; //converte do pino A1
  ADCSRA |= 0b01000000; // inicia a conversao A/D
  while(!(ADCSRA & 0b00010000)); //Aguarda a conversao ser concluida
  return ADC >> 2; //desloca(ignorando os dois bits menos signicativos) para ADC ocupar um byte
}

uint8_t readLumin(){
  ADCSRA |= 0b10000111; //divide o clock_ms por 128 (o clock_ms de conversao sera 16Mhz/128)

  ADMUX   = 0;
  ADMUX  |= 0b01000000;//usa o Vcc como ref
  ADCSRA |= 0b01000000; // inicia a conversao A/D
  while(!(ADCSRA & 0b00010000)); //Aguarda a conversao ser concluida
  return ADC >> 2; //desloca(ignorando os dois bits menos signicativos) para ADC ocupar um byte
}

//Comunicacao SPI
void SPI_MasterInit(void)
{
  PORTB|= 1 << PB2;
  /* Set MOSI and SCK output, all others input */
  DDRB = (1<<DDB3)|(1<<DDB5);
  /* Enable SPI, Master, set clock rate fck/128 */
  SPCR = (1<<SPE)|(1<<MSTR) | (0b00000110);
}
uint8_t SPI_MasterTransmit(uint8_t cData)
{
  /*procedimento nao pertence ao protocolo SPI, mas foi usado para garantir o sincronismos com a FPGA*/
  PORTD|= (1 << LOAD_PIN);
  _delay_us(50);
  PORTD&= ~(1 << LOAD_PIN);
  _delay_us(50);

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
