//#ifdef OPTFLOW

/* Optical Flow mode flag */
static int8_t optflowMode = 0;

/* Angles of correction */
extern int16_t optflow_angle[2];// = { 0, 0 }; //was static

boolean	initOptflow();
void	Optflow_update();
void	optflow_start();
void optflow_end();
void    optflow_read();
void	optflow_get_vel();
void	optflow_get();
uint8_t optflow_squal();

byte read_register(byte address);
void write_register(byte address, byte value);
byte backup_spi_settings();
byte restore_spi_settings();

void rotate16(int16_t *V, int16_t delta);



//endif
