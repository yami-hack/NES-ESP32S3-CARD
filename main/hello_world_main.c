/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <wchar.h>
#include <stdio.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"
#include <math.h>
#include <stdint.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "soc/spi_reg.h"
#include "soc/spi_struct.h"
#include "soc/dport_access.h"
//#include "soc/dport_reg.h"
#include <string.h>

#if 1
#include "driver/i2c.h"
#include "driver/adc.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#endif

#include "esp_task.h"

#include "driver/dedic_gpio.h"


#define DR_REG_INTERRUPT_CORE0_BASE                 DR_REG_INTERRUPT_BASE
#define INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_MAP_REG          (DR_REG_INTERRUPT_CORE0_BASE + 0x040)
/* INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_MAP : R/W ;bitpos:[4:0] ;default: 5'd16 ; */
/*description: */
#define INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_MAP  0x0000001F
#define INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_MAP_M  ((INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_MAP_V)<<(INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_MAP_S))
#define INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_MAP_V  0x1F
#define INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_MAP_S  0
#define INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_NMI_MAP_REG          (DR_REG_INTERRUPT_CORE0_BASE + 0x044)

#define SSD1306
#include "lcd_data.h"

#define READ_REG(REG) (*((volatile uint32_t *)(REG)))
#define WRITE_REG(REG, VAL) *((volatile uint32_t *)(REG)) = (VAL)
#define REG_SET_MASK(reg, mask) WRITE_REG((reg), (READ_REG(reg)|(mask)))
#define REG_CLR_MASK(reg, mask) WRITE_REG((reg), (READ_REG(reg)&(~(mask))))

static portMUX_TYPE s_init_spinlock = portMUX_INITIALIZER_UNLOCKED;

#define INLINE static inline
#define Delay(m) vTaskDelay((m) / portTICK_PERIOD_MS);

#if CONFIG_IDF_TARGET_ESP32
uint32_t IRAM_ATTR gpio_test(){
    uint32_t value = 0;
    int i = 0;
    for(i=0;i<10*1000*1000;i++){
        uint32_t in_value = READ_REG(GPIO_IN_REG);
        value += in_value;
    }
    return value;
}

void IRAM_ATTR gpio_testAll(){
            int t2 = 0;
    int t1 = 0;
    for (int i = 10; i >= 0; i--) {
        t1 = xTaskGetTickCountFromISR();
        uint32_t value = gpio_test();

        //vTaskDelay(1000 / portTICK_PERIOD_MS);
        t2 = xTaskGetTickCountFromISR();
        printf("Restarting in %d seconds...:%08X,%d\n", i,value,t2-t1);
    }
}

#if 0
static const uint8_t gpio_inputs[] = {
    2,4,5,
    13,14,15,16,17,18,19,
    21,22,23,25,26,27,32,
    33,34,35,36,39
};

void test_gpio(){
    //测试GPIO
    for(int i=0;i<sizeof(gpio_inputs);i++){
        int num = gpio_inputs[i];
        gpio_reset_pin(num);
        printf("set pin:%d\n",num);
        gpio_set_pull_mode(num,GPIO_PULLDOWN_ONLY);
        int err = gpio_set_direction(num,GPIO_MODE_INPUT);
        printf("error:%d\n",err);
    }

    for (int i = 600; i >= 0; i--) {
        uint32_t value = READ_REG(GPIO_IN_REG);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        int l = gpio_get_level(2);
        printf("Restarting in %d seconds...:%08X,%d\n",i,value,l);
    }
}
#else
uint8_t  gpio_outputs[] = {
    /*2, //作为输入 */

    4,5,
    13,14,15,16,17,18,19,
    21,22,23,25,26,27,32,
    33,34,35,36,39

    //18
};

void IRAM_ATTR test_gpio(){
    //测试GPIO
    gpio_reset_pin(2);
    gpio_set_pull_mode(2,GPIO_PULLDOWN_ONLY);
    gpio_set_direction(2,GPIO_MODE_INPUT);
    for(int i=0;i<sizeof(gpio_outputs);i++){
        int num = gpio_outputs[i];
        gpio_reset_pin(num);
        printf("set pin:%d\n",num);
        gpio_set_pull_mode(num,GPIO_PULLUP_ONLY);
        int err = gpio_set_direction(num,GPIO_MODE_OUTPUT);
        printf("error:%d\n",err);
    }
    uint32_t set = 0xffffffff;
    uint32_t unset = 0x0;
    for (int i = 600; i >= 0; i--) {
        uint32_t value = READ_REG(GPIO_IN_REG);
        WRITE_REG(GPIO_OUT_REG,(i&1)?set:unset);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        int l = gpio_get_level(2);
        printf("Restarting in %d seconds...:%08X,%d\n",i,value,l);
    }
}
#endif


#define DS 23
#define ST 4
#define SH 18
//随便设置一个片选
#define CS 16
#define NOP /*vTaskDelay(1 / portTICK_PERIOD_MS)*/
uint32_t gpio_value = 0;
static inline IRAM_ATTR void set_gpio(int num,int value){
    #if 1
        if(value){
            REG_SET_MASK(GPIO_OUT_REG,1<<num);
        }
        else{
            REG_CLR_MASK(GPIO_OUT_REG,1<<num);
        }
    #elif 0
        if(value) gpio_value |= (1<<num);
        else      gpio_value &= ~(1<<num);
        REG_WRITE(GPIO_OUT_REG,gpio_value);
    #else
        gpio_set_level(num,value);
    #endif
}

static inline IRAM_ATTR int get_gpio(int num){
    return (READ_REG(GPIO_IN_REG)>>num) & 1;
}

static IRAM_ATTR void _74hc595(uint32_t dat){
    //vTaskDelay(1000 / portTICK_RATE_MS);
    #if 0
    WRITE_REG(GPIO_OUT_REG,dat);
    return;
    #endif
    int i;
    set_gpio(ST,0);
    #if 1
    for(i=0;i<8;i++){
        set_gpio(DS,(dat&0x80)?1:0);
        NOP;
        set_gpio(SH,1);
        NOP;
        set_gpio(SH,0);
        NOP;
        // set_gpio(DS,0);
        // NOP;
        dat<<=1;
    }
    set_gpio(DS,0);
    set_gpio(ST,1);
    #else
    u32 reg_value = READ_REG(GPIO_OUT_REG);
    for(i=0;i<8;i++){
        if(dat&0x80)reg_value|= 1<<(DS);else reg_value &= ~(1<<(DS));
        WRITE_REG(GPIO_OUT_REG,reg_value);
        reg_value |= (1<<SH);  WRITE_REG(GPIO_OUT_REG,reg_value);
        reg_value &= ~(1<<SH); WRITE_REG(GPIO_OUT_REG,reg_value);
        reg_value &= ~(1<<DS); WRITE_REG(GPIO_OUT_REG,reg_value);
    }
    set_gpio(ST,1);
    #endif
}

static IRAM_ATTR void _74hc595_2(uint32_t dat){
    //vTaskDelay(1000 / portTICK_RATE_MS);
    #if 0
    WRITE_REG(GPIO_OUT_REG,dat);
    return;
    #endif
    int i;
    set_gpio(ST,0);
    uint32_t gpio_value = READ_REG(GPIO_OUT_REG);       //获得数据
    uint32_t gpio_h_value =  (gpio_value | (1<<SH)) |  (1<<DS);     //ds位为1
    uint32_t gpio_l_value =  (gpio_value | (1<<SH)) & ~(1<<DS);     //ds位为0
    uint32_t gpio_sh_low_h = ((gpio_value) & ~(1<<SH)) | (1<<DS);     //SH=0,ds=1
    uint32_t gpio_sh_low_l = ((gpio_value) & ~((1<<SH)|(1<<DS)));     //SH=0,ds=1
    #if 1
    #if 1
    /*
    使用两个,则不需要多一个写入?!
    WRITE_REG(GPIO_OUT_REG,(dat&0x80)?gpio_h_value:gpio_l_value);
    WRITE_REG(GPIO_OUT_REG,(dat&0x80)?gpio_sh_low_h:gpio_sh_low_l);
    */
    for(i=0;i<8;i++){
    #else
    //如果使用WRITE_REG(GPIO_OUT_REG,(dat&0x80)?gpio_h_value:gpio_l_value);,则需要多1个写入
    //
    for(i=0;i<=8;i++){
    #endif
        #if 0
        set_gpio(DS,(dat&0x80)?1:0);
        NOP;
        set_gpio(SH,1);
        NOP;
        set_gpio(SH,0);
        NOP;
        #else
        WRITE_REG(GPIO_OUT_REG,(dat&0x80)?gpio_h_value:gpio_l_value);
        WRITE_REG(GPIO_OUT_REG,(dat&0x80)?gpio_sh_low_h:gpio_sh_low_l);
        #endif
        // set_gpio(DS,0);
        // NOP;
        dat<<=1;
    }
    set_gpio(DS,0);
    set_gpio(ST,1);
    #else
    u32 reg_value = READ_REG(GPIO_OUT_REG);
    for(i=0;i<8;i++){
        if(dat&0x80)reg_value|= 1<<(DS);else reg_value &= ~(1<<(DS));
        WRITE_REG(GPIO_OUT_REG,reg_value);
        reg_value |= (1<<SH);  WRITE_REG(GPIO_OUT_REG,reg_value);
        reg_value &= ~(1<<SH); WRITE_REG(GPIO_OUT_REG,reg_value);
        reg_value &= ~(1<<DS); WRITE_REG(GPIO_OUT_REG,reg_value);
    }
    set_gpio(ST,1);
    #endif
}

//使用DMA发送
#define SPI_ID 1
#define SPI_OUTLINK_SET(restart,start,stop,addr) \
    (addr&0xfff)|(restart<<30)|(start<<29)|(stop<<28)
#define SPI_INLINK_SET(restart,start,stop,addr,auto_ret) \
    (addr&0xfff)|(restart<<30)|(start<<29)|(stop<<28)|(auto_ret<<20)


/*

寄存器操作SPI

参考如下
    需要手动把数据写入SPI_W0_REG~SPI_W15_REG寄存器

    发送cmd数据步骤为
        设置mosi_dlen.dbitlen               为0~(16*32-1) 最多发送64字节的数据
        miso_dlen.dbitlen                   为0
        把数据复制到 SPI_W0_REG~SPI_W15_REG  寄存器
        最后调用cmd.usr,等待cmd.usr状态

    发送Transfer
        同发送cmd数据,不过不等待cmd.usr状态

DMA操作SPI
    设置DMA重置,    dma_out->dma_conf.out_rst=1,dma_out->dma_conf.out_rst=0;
    设置fifo重置,   dma_out->dma_conf.dma_afifo_rst=1,dma_out->dma_conf.dma_afifo_rst=0;
    关闭fifo空指针的错误,dma_int_clr.outfifo_empty_err = 1;
    dma_tx开启,     dma_conf.dma_tx_ena = 1
    设置如下数据到dma_out_link.addr , 需要&0xfffff
        设置DMA链接描述符,(默认4KB(LLDESC_MAX_NUM_PER_DESC),但循环添加到链接描述符链表上)
    开始out_link, dma_out_link.start = 1; //开启
    这些寄存器应该在DMA设置后配置
    开启user.usr_mosi = 1;发送数据
        禁用miso,hw->user.usr_miso = 0;
*/

typedef union {
    uint32_t value;
    struct {
            uint32_t clkcnt_l:       6;                     /*it must be equal to spi_clkcnt_N.*/
            uint32_t clkcnt_h:       6;                     /*it must be floor((spi_clkcnt_N+1)/2-1).*/
            uint32_t clkcnt_n:       6;                     /*it is the divider of spi_clk. So spi_clk frequency is system/(spi_clkdiv_pre+1)/(spi_clkcnt_N+1)*/
            uint32_t clkdiv_pre:    13;                     /*it is pre-divider of spi_clk.*/
            uint32_t clk_equ_sysclk: 1;                     /*1: spi_clk is eqaul to system 0: spi_clk is divided from system clock.*/
    };
} spiClk_t;


#define ClkRegToFreq(reg) (apb_freq / (((reg)->clkdiv_pre + 1) * ((reg)->clkcnt_n + 1)))
uint32_t spiFrequencyToClockDiv(uint32_t freq)
{
    uint32_t apb_freq = APB_CLK_FREQ;

    if(freq >= apb_freq) {
        return SPI_CLK_EQU_SYSCLK;
    }

    const spiClk_t minFreqReg = { 0x7FFFF000 };
    uint32_t minFreq = ClkRegToFreq((spiClk_t*) &minFreqReg);
    if(freq < minFreq) {
        return minFreqReg.value;
    }

    uint8_t calN = 1;
    spiClk_t bestReg = { 0 };
    int32_t bestFreq = 0;

    while(calN <= 0x3F) {
        spiClk_t reg = { 0 };
        int32_t calFreq;
        int32_t calPre;
        int8_t calPreVari = -2;

        reg.clkcnt_n = calN;

        while(calPreVari++ <= 1) {
            calPre = (((apb_freq / (reg.clkcnt_n + 1)) / freq) - 1) + calPreVari;
            if(calPre > 0x1FFF) {
                reg.clkdiv_pre = 0x1FFF;
            } else if(calPre <= 0) {
                reg.clkdiv_pre = 0;
            } else {
                reg.clkdiv_pre = calPre;
            }
            reg.clkcnt_l = ((reg.clkcnt_n + 1) / 2);
            calFreq = ClkRegToFreq(&reg);
            if(calFreq == (int32_t) freq) {
                memcpy(&bestReg, &reg, sizeof(bestReg));
                break;
            } else if(calFreq < (int32_t) freq) {
                if(abs(freq - calFreq) < abs(freq - bestFreq)) {
                    bestFreq = calFreq;
                    memcpy(&bestReg, &reg, sizeof(bestReg));
                }
            }
        }
        if(calFreq == (int32_t) freq) {
            break;
        }
        calN++;
    }
    return bestReg.value;
}

uint8_t *dma_mem;
lldesc_t *dma_desc;
static void dma_desc_init(){
    //初始化
    dma_mem = heap_caps_malloc(0x10,MALLOC_CAP_DMA);
    dma_desc = heap_caps_malloc(sizeof(*dma_desc)*0x8,MALLOC_CAP_DMA);
    memset(dma_desc,0,sizeof(*dma_desc)*0x8);
    dma_desc->size = 1;     //1字节
    dma_desc->length = 4;   //4对齐?
    dma_desc->offset = 0;
    dma_desc->sosf = 0;
    dma_desc->eof = 1;
    dma_desc->owner = 1;
    dma_desc->buf = dma_mem;
    dma_desc->empty = 0;
}
static void _74hc595_spi_reg_init(){
    uint8_t unit = 3;

    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_SPI3_CLK_EN);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_SPI3_RST);

    WRITE_REG(SPI_CMD_REG(unit),0);

    //初始化寄存器状态
    WRITE_PERI_REG(SPI_USER_REG(unit), 0);
    WRITE_PERI_REG(SPI_USER1_REG(unit), 0);
    WRITE_PERI_REG(SPI_USER2_REG(unit), 0);
    WRITE_PERI_REG(SPI_CTRL_REG(unit), 0);
    WRITE_PERI_REG(SPI_CTRL2_REG(unit), 0);
    WRITE_PERI_REG(SPI_SLAVE_REG(unit), 0);
    WRITE_PERI_REG(SPI_PIN_REG(unit), 0);
    WRITE_PERI_REG(SPI_CLOCK_REG(unit), 0);
	WRITE_PERI_REG(SPI_W0_REG(unit), 0);

    WRITE_PERI_REG(SPI_W8_REG(unit), 0b0101010101);

    #if 0
    SET_PERI_REG_BITS(IO_MUX_GPIO12_REG, MCU_SEL, 1, MCU_SEL_S);		// GPIO12 -> PadName MTCK -> Function 2: HSPID -> MOSI
	SET_PERI_REG_BITS(IO_MUX_GPIO13_REG, MCU_SEL, 1, MCU_SEL_S);		// GPIO13 -> PadName MTCK -> Function 2: HSPID -> MOSI
	SET_PERI_REG_BITS(IO_MUX_GPIO14_REG, MCU_SEL, 1, MCU_SEL_S);		// GPIO14 -> PadName MTMS -> Function 2: HSPICLK -> CLK
	SET_PERI_REG_BITS(IO_MUX_GPIO15_REG, MCU_SEL, 1, MCU_SEL_S);		// GPIO15 -> PadName MTDO -> Function 2: HSPICS0 -> CS
    #else
    //设置功能function1 设置为VSPI,18时钟,23 MOSI,19 MISO,5 CS
    SET_PERI_REG_BITS(IO_MUX_GPIO19_REG,MCU_SEL,1,MCU_SEL_S);
    SET_PERI_REG_BITS(IO_MUX_GPIO23_REG,MCU_SEL,1,MCU_SEL_S);
    SET_PERI_REG_BITS(IO_MUX_GPIO18_REG,MCU_SEL,1,MCU_SEL_S);
    SET_PERI_REG_BITS(IO_MUX_GPIO5_REG, MCU_SEL,1,MCU_SEL_S);
    #endif

    //SET_PERI_REG_MASK(IO_MUX_GPIO19_REG,FUN_IE);

    #if 0
    //应该全部默认
    SET_PERI_REG_MASK(SPI_PIN_REG(unit),  SPI_CK_IDLE_EDGE); 									// set to 1
	CLEAR_PERI_REG_MASK(SPI_USER_REG(unit), SPI_CK_I_EDGE | SPI_USR_COMMAND);					// cleared to 0
	SET_PERI_REG_BITS(SPI_CTRL2_REG(unit), SPI_MISO_DELAY_MODE, 0, SPI_MISO_DELAY_MODE_S);		// set to 0
	SET_PERI_REG_BITS(SPI_CTRL2_REG(unit), SPI_MISO_DELAY_NUM, 0, SPI_MISO_DELAY_NUM_S);		// set to 0
	SET_PERI_REG_BITS(SPI_CTRL2_REG(unit), SPI_MOSI_DELAY_MODE, 2, SPI_MOSI_DELAY_MODE_S);		// set to 2
	SET_PERI_REG_BITS(SPI_CTRL2_REG(unit), SPI_MOSI_DELAY_NUM, 2, SPI_MOSI_DELAY_NUM_S);		// set to 2
    #endif

    //设置高位模式
    CLEAR_PERI_REG_MASK(SPI_CTRL_REG(unit), SPI_WR_BIT_ORDER | SPI_RD_BIT_ORDER);

    //输出模式
    SET_PERI_REG_MASK(SPI_USER_REG(unit),
    //SPI_USR_MOSI 不使用MOSI
    0
    );

    //设置频率100hz
    WRITE_REG(SPI_CLOCK_REG(unit),spiFrequencyToClockDiv(1));

    SPI3.slave.val = 0; //重置
    SPI3.pin.val = 0;   //重置
    SPI3.user.val = 0;
    SPI3.user1.val = 0;
    SPI3.ctrl1.val = 0;
    SPI3.ctrl2.val = 0;

    SPI3.user.sio = 1;                  //三线半双工
    //SPI3.user.usr_dummy_idle = 1;       //等待时,无时钟输出
    SPI3.user.usr_addr = 0;
    SPI3.user.usr_command = 1;          //用户命令模式
    //SPI3.user.ck_out_edge = 1;
    //SPI3.user.doutdin = 1;

    //SPI3.user.val = 0x80000051;

    //SPI3.user1.usr_addr_bitlen = 0;     //地址数据为0
    SPI3.user2.usr_command_bitlen = 8;  //8位
    SPI3.user2.usr_command_value = 0x1f;    //命令数据
    //WRITE_REG(SPI_CLOCK_REG(unit),0x000070C7);    //频率
    WRITE_REG(SPI_CLOCK_REG(unit),SPI_CLK_EQU_SYSCLK);      //80Mhz
    //WRITE_REG(SPI_USER2_REG(unit),(7<<28)|0x1f);
    // SPI3.mosi_dlen.usr_mosi_dbitlen = 0;    //不需要
    // SPI3.miso_dlen.usr_miso_dbitlen = 0;    //不需要
    printf("init:%X,%08X,%08X\n",SPI3.user2.usr_command_value,READ_REG(SPI_USER2_REG(unit)),SPI3.user.val);

}

static void _74hc595_spi_reg(uint32_t dat){
    //SPI3.user.usr_command = 1;          //用户命令模式
    while(SPI3.cmd.usr);
    SPI3.user2.usr_command_value = dat;
    //printf("send:%X\n",SPI3.user2.usr_command_value);
    SPI3.cmd.usr = 1;
}


spi_device_handle_t spi;
spi_transaction_ext_t t;

static void _74hc595_spi_init(){
    /*初始化*/
    memset(&t,0,sizeof(t));
    t.base.length = 0;
    t.base.user = 1;
}

static IRAM_ATTR void _74hc595_spi(uint32_t dat){
    //static spi_transaction_t trans[1];

    /*
    默认状态下的时序应该是
           | | | | | | | | | | | | | | | |
        SH:__==____==____==____==____==____
        DS:======______======______=====
       DSV:   1    0     1      0    1

    SH在低电平时,需要两个时钟
    */

    //trans[0].flags = SPI_TRANS_USE_TXDATA;
    esp_err_t ret;
    //t.tx_buffer = (uint8_t*)&dat;
    #if 0
    t.base.flags = SPI_TRANS_USE_TXDATA;
    t.base.tx_buffer = (uint8_t*)&dat;
    t.base.length = 8;
    t.base.user = 1;
    #else
    t.base.flags = SPI_TRANS_VARIABLE_CMD;
    t.command_bits = 8;
    t.base.cmd = dat;
    #endif
    gpio_set_level(ST,0);
    #if 1
    ret=spi_device_polling_transmit(spi, &t.base);  //Transmit!
    gpio_set_level(ST,1);
    #else
    ret=spi_device_queue_trans(spi, &t, portMAX_DELAY);
    #endif
    assert(ret==ESP_OK);            //Should have had no issues.
}

static int spi_init = 0;
void test_74hc595_loop(){
    int i = 0;
    #if 0
    gpio_value = READ_REG(GPIO_OUT_REG);
    /*
    使用rand随机数
    1MHz 4s     REG_SET_MASK    //即时读写
    1MHz 2.5s   REG_WRITE       //保存数据,遇到指令时直接写
    1MHz 7.12s  gpio_set_level  //通过api写
    2Mhz 1.22   no 595          //忽略595芯片

    下列使用递增数
    10Mhz 1s    no 595          //忽略595,直接写
    1Mhz  2s    REG_WRITE       //遇到指令时直接写
    1Mhz  3.4s  REG_SETMASK     //
    1Mhz  2s    合并写          //
    1MHz  6.59s gpio_set_level  //通过api写
    1Mhz 10s    spi             //堵塞型
    1Mhz 27s    queue           //提交型
    1Mhz 1.5s   _74hc595_2      //快速设置型
    2Mhz 1.5s   REG SPI wait    //使用REG SPI处理,同步模式
    2Mhz 1.1s   REG SPI         //异步模式,再次写入时等待

    */
    for(i=0;i<1*1000*1000;i++){
        _74hc595((uint32_t)i);
    }
    #elif 0
    if(!spi_init){
        spi_init = 1;
        //for(i=0;i<1*1000*1000;i++){
            _74hc595_spi(0x1f);
        //}
    }
    #elif 0
    for(i=0;i<1*1000*1000;i++){
        _74hc595_2((uint32_t)i);
    }
    #else
    if(!spi_init){
        spi_init = 1;
        //dma_desc_init();
        //设置一次传送
        //dma_mem[0] = 0x1f;

        //初始化
        _74hc595_spi_reg_init();

        set_gpio(ST,0);

        //调用SPI
        _74hc595_spi_reg(0x1f);

        set_gpio(ST,1);
        set_gpio(ST,1);
    }
    for(i=0;i<2*1000*1000;i++){
        set_gpio(ST,0);
        //调用SPI
        _74hc595_spi_reg(0x1f);
        set_gpio(ST,1);
        //set_gpio(ST,1);
    }

    #endif
}

void _callback(spi_transaction_t *t)
{
    printf("set :\n");
    int st=(int)t->user;
    gpio_set_level(ST, st);
}

void test_74hc595(){
    // gpio_reset_pin(DS);
    // gpio_reset_pin(ST);
    // gpio_reset_pin(SH);
    // gpio_reset_pin(17);
    gpio_set_direction(DS,GPIO_MODE_OUTPUT);
    gpio_set_direction(ST,GPIO_MODE_OUTPUT);
    gpio_set_direction(SH,GPIO_MODE_OUTPUT);
    gpio_set_direction(17,GPIO_MODE_OUTPUT);
    gpio_set_direction(15,GPIO_MODE_OUTPUT);
    gpio_set_level(17,0);
    gpio_set_level(15,1);

    uint32_t value;

    #if 0
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num=19,
        .mosi_io_num=DS,
        .sclk_io_num=SH,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz= 0x10,
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=10*1000000,           //Clock out at 10 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=ST,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=_callback,  //Specify pre-transfer callback to handle D/C line
    };

    ret=spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ret=spi_bus_add_device(SPI3_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    _74hc595_spi_init();
    _74hc595_spi(0xf);
    #elif 0
    _74hc595_2(0x1f);
    #endif

    int t1,t2;
    int i;

    for (i = 60; i >= 0; i--) {
        t1 = xTaskGetTickCountFromISR();
        test_74hc595_loop();
        t2 = xTaskGetTickCountFromISR();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("Restarting in %d seconds...:%d,%08X,%08X,%08X,%08X\n",i,t2-t1,READ_REG(SPI_CLOCK_REG(0)),
        READ_REG(SPI_CLOCK_REG(1)),
        READ_REG(SPI_CLOCK_REG(3)),
        READ_REG(SPI_USER_REG(3))
        );
    }
}

#endif //CONFIG_IDF_TARGET_ESP32

static inline uint32_t _getCycleCount(void) {
  uint32_t ccount;
  __asm__ __volatile__("rsr %0,ccount":"=a" (ccount));
  return ccount;
}

static inline uint32_t _getINTENABLE(){
    uint32_t v;
    __asm__ __volatile__("rsr %0,INTENABLE":"=a" (v));
    return v;
}

static inline uint32_t RREADD(uint32_t r){
    uint32_t ret;
    __asm__ __volatile__("l32i %0,%1,0":"=a" (ret) :"r" (r));
    return ret;
}

extern unsigned char nesData[];

float Mhz = 0;
_Atomic int         write_time = 0;
uint32_t    alog_addr[0x100];
_Atomic int         log_addr_set;
_Atomic uint32_t    cur_addr;

typedef enum{
    dedic_in,
    dedic_out,
}dedic_inout_t;

static uint32_t init_dedic(int *gpios,dedic_inout_t t,int gpio_num){
    uint32_t mask = 0;
    dedic_gpio_bundle_handle_t test_bundle = NULL;
    dedic_gpio_bundle_config_t bundle_config = {
            .gpio_array = gpios,
            .array_size = gpio_num,
    };
    if(t==dedic_in){
        bundle_config.flags.out_en = 0;
        bundle_config.flags.in_en = 1;
    }
    else{
        bundle_config.flags.out_en = 1;
        bundle_config.flags.in_en = 0;
    }
    dedic_gpio_new_bundle(&bundle_config, &test_bundle);
    if(t==dedic_in){
        dedic_gpio_get_in_mask(test_bundle, &mask);
    }
    else{
        dedic_gpio_get_out_mask(test_bundle, &mask);
    }
    return mask;
}

#ifndef PACKED
#define PACKED __attribute__ ((packed))
#endif

typedef struct PACKED exnes_rom_header_t{
    /*PRG:N*16K,CHR:N*8K*/
    uint8_t id[4];       //NES
    uint8_t PRG_page;    // N*16K
    uint8_t CHR_page;    // N*8K
    uint8_t vmirror:1;   //0为水平镜像,1为垂直镜像
    uint8_t battery:1;   //存在sram设置
    uint8_t patch:1; //512训练器补丁?
    uint8_t _4screenVam:1;
    uint8_t mapper:4;
    uint8_t vs:1;    //有关街机的ROM
    uint8_t pc10:1;  //有关街机的ROM
    uint8_t zero:2;
    uint8_t mapper2:4;   //如过0Fh为0可以忽略
    uint8_t sram_num;        //sram数量数
    uint8_t zero2;   //09h
    uint8_t zero3;   //0ah
    uint8_t zero4;   //0bh
    uint8_t zero5;   //0ch
    uint8_t zero6;   //0dh
    uint8_t zero7;   //0eh
    uint8_t _0Fh;    //0fh,有关mapper2
}exnes_rom_header_t;

uint8_t *nes_map[2] = {0,0};
volatile int switch_bank = 0;
volatile int switch_addr = 0;
uint8_t nes_addr_lo = 0;       //异步处理地址线不太行.
uint32_t nes_dd = 0;


    extern void esp_intr_enable_source(int inum);
    extern void esp_intr_disable_source(int inum);

IRAM_ATTR void core2_task(){
    int i = 0;
    #if 1

    gpio_reset_pin(42);
    gpio_reset_pin(17);
    gpio_set_direction(42,GPIO_MODE_INPUT);
    gpio_set_direction(17,GPIO_MODE_OUTPUT);

    gpio_reset_pin(21);
    gpio_set_direction(21,GPIO_MODE_INPUT);
    //gpio_set_level(21,0);

    gpio_reset_pin(47);
    gpio_set_direction(47,GPIO_MODE_INPUT);
    for(i=0;i<8;i++){
        gpio_reset_pin(i+9);
        gpio_set_direction(i+9,GPIO_MODE_INPUT);
    }

    int test_gpios[SOC_DEDIC_GPIO_IN_CHANNELS_NUM] = {
        9,10,11,12,
        13,14,15,21     /*触发*/
    };
    //init_dedic(test_gpios,dedic_in,8);
    // test_gpios[0] = 17;
    // test_gpios[1] = 21;
    // init_dedic(test_gpios,dedic_in,9);                             // 4245 /OE ,CPU R/W
    int c = 0;
    /*
    测试(ESP仿真SRAM),1065行修改reset设置的数据线,
    设为0xC2时,跳转0x40C0
    设为0xD2时,跳转0x50D0
    设为0xE0时,跳转0x60E0
    设为0xF8时,跳转0x78F8
    设为0xF4时,跳转0x70F0
    设为0xF5时,跳转0x70F0
    可能是因为引脚问题,
    重新测试0xF7,后跳到77F7
    */

    #define READ_GPIO cpu_ll_read_dedic_gpio_in()
    #define WRITE_GPIO(d) \
            cpu_ll_write_dedic_gpio_all((d))
    #endif

    //esp_intr_disable_source(2);
    #if 0
    esp_intr_disable_source(3);
    esp_intr_disable_source(9);
    esp_intr_disable_source(25);
    esp_intr_disable_source(28);
    #endif
    printf("INTENABLE 2:%08X\n",_getINTENABLE());
    // fflush(stdout);

    //portENTER_CRITICAL_ISR(&s_init_spinlock);

    int old_bank = switch_bank;
    uint8_t *nes_addr_lo_ptr = &nes_addr_lo;
    uint32_t *nes_dd_ptr = &nes_dd;
    while(1){
        #if 1
        if(old_bank!=switch_bank){
            old_bank = switch_bank;
            printf("s:%04X,%06X\n",switch_addr,old_bank>>1);
            fflush(stdout);
        }
        #else
        *(volatile uint32_t*)(nes_dd_ptr) = READ_GPIO;
        //WRITE_REG(nes_dd_ptr,READ_REG(GPIO_IN_REG));
        #endif
    }

}

extern int nes_size;

IRAM_ATTR void test_sram_out(uint8_t *nes_ptr){
    uint32_t log_addr[0x200];
    int log_addr_set = 0;
    int i = 0;
    register uint16_t addr;
    register uint8_t  addr_lo;
    int t1 = 0;
    for(i=0;i<0x100;i++){
        while( (_getCycleCount()-t1<70) ||!((addr=READ_GPIO)&0x80)){
            //80 * (100/24) = 333ns
            //70 * (100/24) = 291ns
            //60 * (100/24) = 250ns
            //需要等待高电平
        }

        while(((addr=READ_GPIO)&0x80)){
                //等待低电平
        }

        t1 = _getCycleCount();
        while(_getCycleCount()-t1<40){
            //80                         不行
            //71                         不行
            //70                         同60,不稳定
            //60                         最稳定,能进等待结束,不慢
            //50 * (100/24) = 208.333ns  最稳定2,能进等待结束 ,但慢一拍
            //40 * (100/24) = 166.666ns  最稳定2,能进等待结束画面
            //30 * (100/24) = 125ns
            //20 * (100/24) = 83.3ns    最稳定
            //10 * (100/24) = 41.666ns  不行
            addr = READ_GPIO;
        }

        addr_lo = READ_REG(GPIO_IN_REG)>>9;
        //addr = READ_GPIO;
        addr <<= 8;
        addr |= addr_lo;
        addr &= 0x7fff;

        uint8_t d = nes_ptr[addr&0x7fff];
        WRITE_GPIO(d);

        log_addr[i] = addr;
    }
    printf("7FFC,7FFD,\n");
    for(i=0;i<0x100;i++){
        printf("%04X,",log_addr[i]);
        if(!(i&0xf)){
            printf("\n");
        }
    }
}

static volatile int set_time = 0;
static volatile int clr_time = 0;
volatile int isr_time = 0;
int *ddd = 0;
typedef uint8_t *nes_mem[4];
static void blank(void*arg){
    //isr_time = _getCycleCount();
    int *x = arg;
    x[0]++;
    //ddd[0]++;
    vTaskDelay(500 / portTICK_PERIOD_MS);
}

IRAM_ATTR void test_phi2(){
    gpio_reset_pin(18);
    gpio_set_direction(18,GPIO_MODE_INPUT);
    int count = 0;
    uint32_t xx = 0;
    uint32_t t1 = _getCycleCount();
    while(1){
        uint32_t a = READ_REG(GPIO_IN_REG);
        if((a&(1<<18))!=(xx&(1<<18))){
            count++;
        }
        xx = a;
        if(_getCycleCount()-t1>240*1000*1000){
            printf("cc:%d\n",count);
            count = 0;
            t1 = _getCycleCount();
        }
    }
}

IRAM_ATTR void test_sram(){
    int i;
    uint32_t mask = 0,mask2 = 0;
    uint32_t t1 = 0,t2 = 0;

    //d0~d7
    // for(i=1;i<=21;i++){
    //     gpio_reset_pin(i);
    //     gpio_set_pull_mode(i,GPIO_PULLDOWN_ONLY);
    //     gpio_set_direction(i,GPIO_MODE_INPUT_OUTPUT);
    // }

    /*
    电流测试
    使用ESP供电,是0.29A
    无卡, 0.23A
    全集成卡带, 0.27A
    半集成卡带, 0.23A
    */

    //#define DUMP_LOG  /*reset导出记录*/
    //#define DUMP_LOG2
    #define _WINDOW_
    //#define TEST_INTERRUPT
    //#define USE_FLASH_DATA  /*使用flash数据 ,如果CPU缓存命中失败,会导致无法进行游戏*/

    for(i=1;i<=8;i++){
        gpio_reset_pin(i);
        gpio_set_pull_mode(i,GPIO_PULLDOWN_ONLY);
        gpio_set_direction(i,GPIO_MODE_INPUT_OUTPUT);
    }

    for(i=9;i<=18;i++){
        gpio_reset_pin(i);
        gpio_set_pull_mode(i,GPIO_PULLDOWN_ONLY);
        gpio_set_direction(i,GPIO_MODE_INPUT);
    }

    for(i=35;i<=42;i++){
        gpio_reset_pin(i);
        gpio_set_pull_mode(i,GPIO_PULLDOWN_ONLY);
        gpio_set_direction(i,GPIO_MODE_INPUT);
        gpio_set_pull_mode(i,GPIO_PULLDOWN_ONLY);
    }

    {
        //初始化专用gpio指令
        int test_gpios[SOC_DEDIC_GPIO_IN_CHANNELS_NUM] = {0};
        int test2_gpios[SOC_DEDIC_GPIO_OUT_CHANNELS_NUM] = {0};
        for(i=0;i<8;i++){
            test_gpios[i] = 35+i;       //GPIO35~GPIO42  a8~a15
            test2_gpios[i] = 1+i;       //GPIO1~GPIO8    d0~d7
        }

        //test_gpios[7] = 18;             //M2时钟线
        //test_gpios[6] = 42;             //A15
        //test_gpios[7] = 18;               //
        #ifdef TEST_INTERRUPT
        test_gpios[0] = 9;                //A9
        #endif


        init_dedic(test_gpios,dedic_in,8);
        init_dedic(test2_gpios,dedic_out,8);
        for(i=0;i<8;i++){
            printf("%X\n",test_gpios[i]);
        }
        printf("%08X,%08X\n",mask,mask2);
    }

    #define ROM_SIZE (128*1024)

    uint8_t *alloc_data = calloc(ROM_SIZE+0x10,1);   //32KB + 0x20
    memcpy(alloc_data,nesData+0x10,ROM_SIZE);

    uint8_t *nes_ptr
        #ifdef USE_FLASH_DATA
        = nesData + 0x10;
        #else
        = alloc_data;
        #endif
    register int addr_mask = 0x3fff;
    exnes_rom_header_t *head = nesData;

    nes_mem nes_map;
    uint8_t * mmap_nes[0x100];
    nes_map[0] = nes_ptr;
    nes_map[1] = nes_ptr + (head->PRG_page-1) * 0x4000;;
    nes_map[2] = nes_map[0];
    nes_map[3] = nes_map[1];
    if(nes_size>=0x5000){
        addr_mask = 0xffff;
    }

    for(i=0;i<(0x100);i+=2){
        mmap_nes[i+0] = nes_ptr + ((i>>1)&0x7) * 0x4000;
        mmap_nes[i+1] = nes_ptr + ((i>>1)&0x7) * 0x4000;
    }

    i = 0;

    #define READ_GPIO2 READ_REG(GPIO_IN_REG)

    #define WRITE_GPIO3(d) \
        WRITE_REG(GPIO_OUT_REG,d<<1);

    #define WRITE_GPIO2(d) \
            WRITE_REG(GPIO_OUT_W1TC_REG,0xff<<1 ); \
            WRITE_REG(GPIO_OUT_W1TS_REG,((d)&0xff)<<1);
    #if 1
    //测试gpio反应速度

    gpio_set_direction(18,GPIO_MODE_INPUT);

    gpio_reset_pin(21);
    gpio_set_pull_mode(21,GPIO_PULLDOWN_ONLY);
    gpio_set_direction(21,GPIO_MODE_INPUT_OUTPUT);
    do{
        printf("GPIO21:%d\n",gpio_get_level(21));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }while(0);
    // gpio_set_level(21,0);


    register uint16_t addr;
    register uint8_t  addr_lo;
    register int16_t old_addr = 0;
    #ifdef DUMP_LOG
    uint32_t  log_addr[0x300];// = calloc(0x400,4);
    uint32_t       log_addr_set = 0;
    int        dump_log = 0;
    #endif

    //s_init_spinlock;

    #ifdef TEST_INTERRUPT
    /*中断触发消息*/

    gpio_config_t c;
    memset(&c,0,sizeof(c));
    c.intr_type = GPIO_INTR_POSEDGE;
    c.pin_bit_mask = (1<<9);
    c.pull_up_en = 1;
    c.mode = GPIO_MODE_INPUT;
    gpio_config(&c);


    static int count = 0;
    WRITE_GPIO(0x0);

    #if 1
    //测试禁用中断源

    #endif

    #if 1
    //测试中断翻转时间
    /*
    WRITE_GPIO(0x0)到触发中断需要
        31个时钟,124ns
    */
   #if 1
    gpio_install_isr_service(ESP_INTR_FLAG_NMI);
    //gpio_install_isr_service(ESP_INTR_FLAG_LEVEL5);
    gpio_isr_handler_add(9,blank,&count);
    #else

    WRITE_REG(
        INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_MAP_REG,
        //INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_NMI_MAP_REG,
        14);

    WRITE_REG(GPIO_PIN9_REG,0x2080);
    esp_intr_enable_source(14);
    #endif
    while(1){
        //uint32_t ireg = READ_REG(INTERRUPT_CORE0_GPIO_INTERRUPT_PRO_MAP_REG);
        //uint32_t ireg = READ_REG(GPIO_PIN9_REG);
        uint32_t ireg = _getINTENABLE();            //默认1200020C,LV3:1280020C,LV2:1208020C,LV1:1200120C
        clr_time = _getCycleCount();
        WRITE_GPIO(0x1);
        set_time = _getCycleCount();
        while(_getCycleCount()-clr_time< 240*1000*1000){
        }
        WRITE_GPIO(0x0);
        int intr_time = isr_time - clr_time;
        printf("count:%d, SCI:%d,%d,-----%dns,%08X,PC:%08X\n",count,set_time-clr_time,isr_time-clr_time,intr_time * (100/24),ireg,isr_time);
    }
    #else
    //不中断,测试翻转时间
    /*
      耗时      时钟    时间            x值
    READ_REG  :  27     108ns           0 (理论上应该更短),
    READ_GPIO :  11     44ns            1 (x++执行了一次,理论上应该在44ns以下)
    */
    while(1){
        register int x = 0;
        clr_time = _getCycleCount();
        WRITE_GPIO(0x0);
        #if 0
        while(READ_REG(GPIO_IN_REG)&(1<<9)){
            x++;
        }
        #else
        while(READ_GPIO&1){
            x++;
        }
        #endif
        isr_time = _getCycleCount();

        set_time = _getCycleCount();
        while(_getCycleCount()-clr_time< 240*1000*1000){
        }
        WRITE_GPIO(0x1);
        int intr_time = isr_time - clr_time;
        printf("count:%d, SCI:%d,%d,-----%dns,%d\n",count,set_time-clr_time,isr_time-clr_time,intr_time * (100/24),x);
    }
    #endif


    return;
    #endif

    esp_intr_disable_source(2);
    esp_intr_disable_source(3);
    esp_intr_disable_source(9);
    esp_intr_disable_source(25);
    esp_intr_disable_source(28);
    printf("INTENABLE:%08X\n",_getINTENABLE());
    fflush(stdout);

    #ifndef DUMP_LOG
    portENTER_CRITICAL_ISR(&s_init_spinlock);
    #endif
    WRITE_GPIO(0xff);

    #ifdef DUMP_LOG2

    while(1){
        for(i=0;i<0x200;i++){
            do{
                addr_lo = (READ_REG(GPIO_IN_REG)>>9) & 0xff ;
            }while((addr=READ_GPIO)&0x80);       //15是高电平
            //t2 = t1;
            t1 = _getCycleCount();
            //a15是低电平
            addr =  (addr<<8) + addr_lo;
            log_addr[i] = addr;
        }
        printf("\n----------\n");
        for(i=0;i<0x200;i++){
            printf("%04X,",log_addr[i]);
            if((i&0xf)==0){
                printf("\n");
            }
        }
    }
    #endif
    i = 0;

    uint8_t *nes_addr_lo_ptr = &nes_addr_lo;
    uint32_t *nes_dd_ptr = &nes_dd;
    uint32_t insn_log = 0;

    while(1){

        /*

        6502时序
            http://archive.6502.org/datasheets/mos_6501-6505_mpu_preliminary_aug_1975.pdf
        lvc4245a
            https://assets.nexperia.cn/documents/data-sheet/74LVC4245A.pdf

        4245(OE)    /```````````````````````````\___________/````````\__________/
        PHI2        _________/`````````\________/```````````\________/``````````\
        PHI1        /3`````4\___________/``````\_____________/``````\__________/`
        ADDR(15)    ____________________________/```````````\________/``````````\
        address     xx/````````````````\xx/`````````````````\xx/````````````````\
        dat (R:65)  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx/````|1\xxxxxxxxxxxx/````|1\
        dat (R:245) xxxxxxxxxxxxxxxxxxxxxxxxxxxx/``````````|2\xxxxxxx/`````````|2\
        dat         zzzzzzzzzzzzzzzzzzzzzzzzzzzz/`$ff````````\zzzzzzz/`$ff```````\

        nes(phi2)   __209ns__/`350ns```\________/```````````\________/``````````\
        nes(dat读取)__________/\_________________/\______________________________
                      279.5ns
                      1到7/8个PPU周期,


        /`````\  高电平(和有效数据)
        \_____/  低电平
        xxxxxx   非有效

        注|1:6502的Th时间(PHI2从高电平到低电平后的持续时间)  ,10~??ns (一般为30ns),
             TDSU时间最小为100ns.(PHI2低电平往前100ns)

        注|2:4245的时间范围 ,
            低电平到Z状态延迟时间 toA:1ns<(2.9ns)<7.0ns     toB:1ns<(3.9ns)<7.7ns
            高电平到Z状态延迟时间 toA:1ns<(2.8ns)<5.8ns     toB:1ns<(2.9ns)<7.8ns
            Z到低电平时间        toA:1ns<()<9.0ns           toB:1ns<()<8.1ns
            Z到高电平时间        toA:1ns<()<8.1ns           toB:1ns<()<8.7ns
        注3,4:  PHI1上升时间和下降时间最大   25ns
                与PHI2的相位时间最小为 0ns






        */

       /*不使用时钟线,使用地址线来获得数据*/
       /* CPU A15线是与地址线15与PHI2与非操作  ~A15 | ~PHI2
            (A15)  (PHI2)
            0       0       ->1
            1       0       ->1
            0       1       ->1
            1       1       ->0
       */
      uint32_t dd;
        /*
        (注, ?<200ns<300ns 为 MIN<TYP<MAX,三个分别为最小值<标准值<最大值)
        数据时序
        上一次读取数据地址  5E70
        a0~a7   为目标写入地址
        a8~a15  为ROM的高位地址
        CPU R/W 为低电平
                  ______                ___________________
        PHI2            \____209ns_____/    350ns          \___  (4245转换,5ns以内)
                        |<>|  ?<200ns<300ns
                        |<>|______________________________
        地址线           xx/
                        |<>|  ?<100ns<300ns
                        |<>|___________
        CPU R/W         xxx/           \


        仿真模式(MCU获得数据时序)
                  ______                ___________________
        PHI2            \____209ns_____/    350ns          \___  (4245转换,5ns以内)
                                       ||_______________________________  1ns<10ns<15ns   注,因为4245的关系,需要附加最多5ns等待. 内部集成LS139来处理a15地址线,所以最多等待10ns
        a8-a15                      xxxx/                                (代码是根据a15低电位时获得的,所以a8~a15是实时处理)
                             |<_______>|______________________________(READ_REG(GPIO_IN_REG),需要15 clock (60ns)  所以应该是 5ns<10ns<60ns
        (a0~a7,cpu r/w):xxxxx/         提前模式获得数据,
        a0~a7,cpu r/w 获得数据的时间应该是
            T(a0~a7) = T(a8~a15) - 15ns - 60ns
            也就是PHI2下降沿时的 75ns后




        */

       do{
           //asm volatile("memw\n");
           dd = READ_REG(GPIO_IN_REG);
           //addr_lo = *(volatile uint8_t*)nes_addr_lo_ptr;
       }while((addr=READ_GPIO)&0x80);       //15是高电平
       //dd = READ_REG(nes_dd_ptr);

       //t2 = t1;
       //t1 = _getCycleCount();
       //a15是低电平
       //addr_lo = nes_addr_lo;
       #if 0
       //不行,速度跟不上
       asm volatile("memw\n");
       dd = *(uint32_t*)GPIO_IN_REG;
       #endif

       #if 0
       uint8_t d = nes_ptr[addr];
       #else
       //addr &= ;

       #endif

#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

       {
            addr_lo = dd>>9;
            addr =  (addr<<8) + addr_lo;
            uint8_t d = nes_map[addr>>14][addr&0x3fff];
            WRITE_GPIO(d);
       }

       #if 1

       uint8_t d;
       //d = *(volatile uint8_t*)nes_dd_ptr;
       if(
           (
            #if 0
            //魂斗罗的更改blank地址
            (old_addr==0x4144)&&(addr==0x7FD1)
            #else
            //测试写入指令
            //(d&(1<<7))
            (dd&(1<<21))
            //&&((insn_log&0xff0000)!=0x8d0000)
            &&(addr>0x7d00)
            #endif
           )
       ){
           //d <<= 1;
           d = dd;
           uint8_t *rom_bin = mmap_nes[d];
           #if 1
           switch_bank = (addr<<9)|d;
           switch_addr = old_addr;
           #endif
            nes_map[0] = rom_bin;
            nes_map[2] = rom_bin;
           //printf("addr:%04X,%08X\n",addr,READ_REG(GPIO_IN_REG) );
       }
       else{
           //old_addr = addr;
       }
       #endif




       while(!(READ_GPIO&0x80)){    //当前低电平,等待高电平
       }

       #ifdef DUMP_LOG
       if(dump_log){
           log_addr[i++] = addr;
           if(i>0x100){
               int x;
               printf("\n----------\n");
               for(x=0;x<0x100;x++){
                   printf("%04X,",log_addr[x]);
                   if((x&0xf)==0){
                       printf("\n");
                   }
               }
               dump_log = 0;
           }
       }

       //gpio为高电平
       if(old_addr==0x7ffc&&addr==0x7ffd){
           i = 2;
           dump_log = 1;
       }
       #endif
    }
    #endif
}

#define SRAM_WE 16
#define SRAM_LB 15
#define SRAM_OE 14
#define SRAM_CS 13
#define SRAM_UB 17
#define CPU_WE  18

#define HC595_CLR  9
#define HC595_CLK  10
#define HC595_RCLK 11
#define HC595_OE   12
#define HC595_SER  18

#define HC595_EXT

void hc595_init(){
    WRITE_REG(GPIO_OUT_W1TC_REG, (1<<HC595_CLK)  | (1<<HC595_RCLK) | (1<<HC595_OE) );
    //覆盖寄存器与清除寄存器
    WRITE_REG(GPIO_OUT_W1TS_REG, (1<<HC595_RCLK) | (1<<HC595_CLR)  );
}

void hc595_set_addr(volatile uint32_t addr){
    //清除位移寄存器,和时钟线
    #define HC595_WAIT /* vTaskDelay(1 / portTICK_PERIOD_MS); */
    WRITE_REG(GPIO_OUT_W1TC_REG, (1<<HC595_CLR) | (1<<HC595_CLK) ); HC595_WAIT ;
    WRITE_REG(GPIO_OUT_W1TS_REG, (1<<HC595_CLR) ); HC595_WAIT; //拉高引脚
    int i;
    for(i=0;i<24;i++){
        WRITE_REG(GPIO_OUT_W1TC_REG,1<<HC595_CLK);HC595_WAIT;  //下降沿
        WRITE_REG((addr&(1<<23))?GPIO_OUT_W1TS_REG:GPIO_OUT_W1TC_REG,1<<HC595_SER);HC595_WAIT;   //设置数据
        WRITE_REG(GPIO_OUT_W1TS_REG,1<<HC595_CLK);HC595_WAIT;  //上升沿
        addr <<= 1;
    }
    WRITE_REG(GPIO_OUT_W1TC_REG, (1<<HC595_RCLK) );HC595_WAIT; //覆盖
    WRITE_REG(GPIO_OUT_W1TS_REG, (1<<HC595_RCLK) );HC595_WAIT; //
}

uint16_t test_sram_read(uint32_t addr){
    //设置地址
    #if defined(HC595_EXT)
        hc595_set_addr(addr);
    #else
        #if 1
        uint32_t dat = READ_REG(GPIO_OUT_REG);
        dat &= ~(0xf<<9);
        dat |= (addr<<9);
        WRITE_REG(GPIO_OUT_REG,dat);
        #else
        WRITE_REG(GPIO_OUT_W1TC_REG,(0xf<<9)  );
        WRITE_REG(GPIO_OUT_W1TS_REG,(addr<<9) );
        #endif
    #endif
    //WRITE_REG(GPIO_OUT_W1TC_REG,(1<<SRAM_OE) ); //开启OE
    vTaskDelay(1 / portTICK_PERIOD_MS);

    uint8_t dat1 = (READ_REG(GPIO_IN_REG) >>1)&0xff;
    uint8_t dat2 = (READ_REG(GPIO_IN_REG) >>1)&0xff;
    //WRITE_REG(GPIO_OUT_W1TS_REG,(1<<SRAM_OE));  //关闭OE
    return dat1 | (dat2<<8);
}

int write_firset = 0;
void test_sram_write(uint32_t addr,uint8_t dat){
    volatile int x = 0;
    #define SET_ADDR \
        WRITE_REG(GPIO_OUT_W1TC_REG,(0xf<<9) ); \
        WRITE_REG(GPIO_OUT_W1TS_REG,(addr<<9));
    #define SET_DAT \
        WRITE_REG(GPIO_OUT_W1TC_REG,(0xff<<1) ); \
        WRITE_REG(GPIO_OUT_W1TS_REG,(dat<<1) );
    #if 0
    //设置地址与数据
    //OE高电平
    WRITE_REG(GPIO_OUT_W1TC_REG,(0xf<<9) | (0xff<<1) | (1<<SRAM_CS) );
    WRITE_REG(GPIO_OUT_W1TS_REG,(addr<<9) | (dat<<1) | (1<<SRAM_OE) );
    //写入数据
    WRITE_REG(GPIO_OUT_W1TC_REG,(1<<SRAM_WE) /* | (1<<SRAM_LB) */ );
    WRITE_REG(GPIO_OUT_W1TS_REG,(1<<SRAM_WE) /* | (1<<SRAM_LB) */ );
    #elif 0
    //方案1
    SET_ADDR
    SET_DAT
    WRITE_REG(GPIO_OUT_W1TC_REG,(1<<SRAM_CS)|(1<<SRAM_WE)|(1<<SRAM_LB));
    WRITE_REG(GPIO_OUT_W1TS_REG,(1<<SRAM_CS)|(1<<SRAM_WE)|(1<<SRAM_LB));

    #elif 0

    volatile int x = 0;
    //方案2
    WRITE_REG(GPIO_OUT_W1TC_REG,(0xf<<9) | (0xff<<1)  );
    WRITE_REG(GPIO_OUT_W1TS_REG,(addr<<9) | (dat<<1)  );
    //vTaskDelay(1 / portTICK_PERIOD_MS);
    //WRITE_REG(GPIO_OUT_W1TC_REG,(1<<SRAM_LB));
    int t1 = _getCycleCount();
    x++;
    x++;
    WRITE_REG(GPIO_OUT_W1TC_REG,(1<<SRAM_WE));
    //vTaskDelay(1 / portTICK_PERIOD_MS);
    int t2 = _getCycleCount();
    x++;
    x++;
    WRITE_REG(GPIO_OUT_W1TS_REG,(1<<SRAM_WE));
    //WRITE_REG(GPIO_OUT_W1TS_REG,(1<<SRAM_LB));
    int t3 = _getCycleCount();
    x++;
    x++;

    printf("cc:%d,%d,%d\n",t2-t1,t3-t2,x);
    #elif 0
    //

    #else
    //能用，但数据有些错误
    //数据有些错误
    int i;
    //设置地址线

    #define WAIT x++;x++;x++;x++; x++;x++;x++;x++;x++;x++;x++;x++; x++;x++;x++;x++;
    #if defined(HC595_EXT)
        WRITE_REG(GPIO_OUT_W1TC_REG,(0xff<<1) );
        WRITE_REG(GPIO_OUT_W1TS_REG,(dat <<1)  );    //设置数据
        hc595_set_addr(addr);                       //设置地址
        WAIT
    #else
        //!HC595_EXT
        #if 0
        for(i=0;i<4;i++){
            gpio_set_level(i+9,addr&1);
            addr >>= 1;
        }
        //设置数据线
        for(i=0;i<8;i++){
            gpio_set_level(i+1,dat&1);
            dat >>= 1;
        }

        #else

        uint32_t a = READ_REG(GPIO_OUT_REG);
        a &= ~((0xf<<9)|(0xff<<1));
        a |= (addr<<9) |(dat<<1);
        WRITE_REG(GPIO_OUT_REG,a); WAIT;
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
        #endif
    #endif //HC595_EXT

    //NOTE4
    /*
    addr :  xxxx-xxxxxxx-xxxxxxx-xxxxx
    oe   :  ---\______________________
    cs1  :  __________________________
    cs2  :  --------------------------
    we   :  ---\________________/-----
    ub,lb:  -------\___/----\___/-----
    din  :  --------\_/------\_/------

    */
    WRITE_REG(GPIO_OUT_W1TC_REG,(1<<SRAM_WE) | (1<<CPU_WE) ); WAIT
    WRITE_REG(GPIO_OUT_W1TS_REG,(1<<SRAM_WE) | (1<<CPU_WE) ); WAIT



    printf("%d\n",x);

    #endif
}

void test_sram_rw(){
    int i;
    for(i=1;i<=8;i++){
        gpio_reset_pin(i);
        //gpio_set_pull_mode(i,GPIO_PULLDOWN_ONLY);
        gpio_set_direction(i,GPIO_MODE_INPUT);
    }

    for(i=9;i<=18;i++){
        gpio_reset_pin(i);
        gpio_set_pull_mode(i,GPIO_PULLDOWN_ONLY);
        gpio_set_direction(i,GPIO_MODE_OUTPUT);
    }

    i = 0;
    hc595_init();

    #if 0
    while(1){
        hc595_set_addr(i+=0x10000);
        vTaskDelay(1 / portTICK_PERIOD_MS);
        uint8_t dat = READ_REG(GPIO_IN_REG)>>1;
        printf("%06X:%02X\n",i-0x10000,dat);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    #endif

    //开启芯片
    gpio_set_level(SRAM_OE,1);  //禁用输出
    //开启LB
    gpio_set_level(SRAM_LB,0);
    gpio_set_level(SRAM_CS,0);
    gpio_set_level(SRAM_UB,1);
    gpio_set_level(SRAM_WE,1);
    gpio_set_level(CPU_WE,0);       //B->A;

    uint16_t addr = 0;

    #if 0
    for(i=1;i<=8;i++){
        gpio_set_direction(i,GPIO_MODE_OUTPUT);
    }
    // volatile int x = 0;
    // //写操作
    // WRITE_REG(GPIO_OUT_W1TC_REG,(0xf<<9)|(0xff<<1));
    // WRITE_REG(GPIO_OUT_W1TS_REG,(0x0<<9)|(0x5<<1));
    // gpio_set_level(SRAM_OE,1);  //禁用输出

    for(i=0;i<0xff;i++){
        #if 1
        if(i==0xf){
            test_sram_write(i,0xf0);
        }
        else
            test_sram_write(i,i);
        #elif 0
        WRITE_REG(GPIO_OUT_W1TC_REG,(1<<SRAM_WE)|(1<<SRAM_LB));
        x++;
        x++;
        x++;
        x++;
        WRITE_REG(GPIO_OUT_W1TS_REG,(1<<SRAM_WE)|(1<<SRAM_LB));
        #else
        gpio_set_level(SRAM_WE,0);  //
        gpio_set_level(SRAM_WE,1);  //
        #endif
    }
    #endif

    gpio_set_level(SRAM_LB,0);
    gpio_set_level(SRAM_OE,0);  //开启输出
    gpio_set_level(SRAM_WE,1);  //禁止写入
    gpio_set_level(SRAM_UB,1);
    gpio_set_level(SRAM_CS,0);
    gpio_set_level(CPU_WE,1);       //A->B

    //必须设置为输入
    for(i=1;i<=8;i++){
        gpio_reset_pin(i);
        //gpio_set_pull_mode(i,GPIO_PULLDOWN_ONLY);
        gpio_set_direction(i,GPIO_MODE_INPUT);
    }

    while(1){
        uint16_t dat = 0;
        if(addr==0){
            printf("wait...\n");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        dat = test_sram_read(addr);
        printf("%04X:%04X\n",addr,dat);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        addr ++;
        addr &= 0xff;
    }
}

void test_nes(){
    int i;
    uint8_t *nes_ptr = nesData + 0x10;
    //重置gpio
    for(i=0;i<=16;i++){
        gpio_reset_pin(i);
        gpio_set_direction(i,GPIO_MODE_INPUT);
        gpio_set_pull_mode(i,GPIO_PULLDOWN_ONLY);
    }

    gpio_reset_pin(21);
    gpio_set_direction(i,GPIO_MODE_INPUT);
    gpio_set_pull_mode(i,GPIO_PULLDOWN_ONLY);

    gpio_reset_pin(17);
    gpio_set_direction(i,GPIO_MODE_OUTPUT);
    //低电平触发中断,需要保持为1;
    gpio_set_level(17,1);

    //d0-d7
    for(i=1;i<=8;i++){
        gpio_set_direction(i,GPIO_MODE_INPUT_OUTPUT);
    }

    //a0~a7
    for(i=9;i<=16;i++){
        //已经初始化
    }
    for(i=35;i<=42;i++){
        gpio_reset_pin(i);
        gpio_set_direction(i,GPIO_MODE_INPUT);
        //需要上拉才行
        gpio_set_pull_mode(i,GPIO_PULLUP_ONLY);
    }

    #define portDISABLE_INTERRUPTS() \
    do { XTOS_SET_INTLEVEL(XCHAL_EXCM_LEVEL); \
        portbenchmarkINTERRUPT_DISABLE();\
    } while (0)

    //portDISABLE_INTERRUPTS();
    int c = 0;
    int time_set = 0;
    uint32_t old_addr = 0;
    uint32_t addr;
    uint16_t addrs[0x10];
    uint32_t addr_set = 0;
    #define AA_DAT 0

    // do{
    //     addr = READ_REG(GPIO_IN1_REG) & 0x7F8;
    // }while(addr&0x400);     //需要先等待低电平
    // do{
    //     addr = READ_REG(GPIO_IN1_REG) & 0x7F8;
    // }while(!(addr&0x400));     //等待高电平

    //IRQ中断
    WRITE_REG(GPIO_OUT_W1TC_REG,1<<17);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    WRITE_REG(GPIO_OUT_W1TS_REG,1<<17);
    gpio_set_direction(17,GPIO_MODE_INPUT);

    while(1){
        // do{

        // }while(!(addr&0x400));     //高电平
        addr = READ_REG(GPIO_IN1_REG) & 0x7F8;
        addr <<= 5;
        addr |= (READ_REG(GPIO_IN_REG)>>9)&0xff;
        //}while(old_addr==addr);
        old_addr = addr;
        uint16_t d = nes_ptr[addr&0x3fff];
        // if((addr&0x400)){
        //     //a15为高电平
        //     //全部置为0
        //     WRITE_REG(GPIO_OUT_W1TC_REG,0xff<<1);
        // }
        // else
        d |= 1<<16;
        {
            WRITE_REG(GPIO_OUT_REG,d<<1);
            //WRITE_REG(GPIO_OUT_REG,0x88<<1);
        }


        addrs[addr_set] = addr;
        if(addr_set>0x10){
            addr_set = 0;
            printf("%04X:%04X,%04X,%04X,%04X,%04X,%04X,%04X,%04X,%04X,%04X,%04X,%04X,%04X,%04X,%04X,%04X,\n",
            c++,
            addrs[0+0],addrs[0+1],addrs[0+2],addrs[0+3],
            addrs[4+0],addrs[4+1],addrs[4+2],addrs[4+3],
            addrs[8+0],addrs[8+1],addrs[8+2],addrs[8+3],
            addrs[12+0],addrs[12+1],addrs[12+2],addrs[12+3]
            );
        }

        //vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}



#define TEST_OUT_DATA 1
#include "test_data.h"
uint8_t nes_data[8] = {
    [7] = 0x3,
    [4] = 0x2,
    [1] = 0x1
};

//输出数据

//测试时钟
void test_clk(){

    #if !CONFIG_IDF_TARGET_ESP32C3
    int i;

    gpio_set_direction(4,GPIO_MODE_OUTPUT);
    gpio_set_level(4,0);
    gpio_set_direction(4,GPIO_MODE_INPUT);
    int t2 = 0;
    int t1 = 0;
    int x = 0;
    uint32_t value = 0;

    #if !TEST_OUT_DATA
    gpio_set_direction(25,GPIO_MODE_OUTPUT);
    gpio_set_direction(26,GPIO_MODE_OUTPUT);
    gpio_set_direction(27,GPIO_MODE_OUTPUT);
    gpio_set_direction(16,GPIO_MODE_INPUT);
    gpio_set_direction(17,GPIO_MODE_INPUT);
    #else
    //关闭中断
    #if 0
    #define portDISABLE_INTERRUPTS() \
    do { XTOS_SET_INTLEVEL(XCHAL_EXCM_LEVEL); \
        portbenchmarkINTERRUPT_DISABLE();\
    } while (0)

    //portDISABLE_INTERRUPTS();
    #endif
    //vTaskEnterCritical(&s_init_spinlock);


    // gpio_set_direction(25,GPIO_MODE_INPUT);
    // gpio_set_direction(26,GPIO_MODE_INPUT);
    // gpio_set_direction(27,GPIO_MODE_INPUT);
    for(i=0;i<=21;i++){
        gpio_reset_pin(i);
        gpio_set_direction(i,GPIO_MODE_INPUT);
        gpio_set_pull_mode(i,GPIO_PULLDOWN_ONLY);
    }

    /*
    输出可行
    17,18,19,20,21
    1,2,3,4,5,6,7,8

    */

    #define TEST_GPIO_MIN 8
    #define TEST_GPIO_MAX 8

    for(i=TEST_GPIO_MIN;i<=TEST_GPIO_MAX;i++){
        gpio_set_direction(i,GPIO_MODE_OUTPUT);
    }

    for(i=39;i<=42;i++){
        //if(i!=41){
            gpio_reset_pin(i);
            gpio_set_direction(i,GPIO_MODE_INPUT_OUTPUT);
            gpio_set_pull_mode(i,GPIO_PULLUP_ONLY);
        //}
    }

    for(i=35;i<=38;i++){
        gpio_reset_pin(i);
        gpio_set_direction(i,GPIO_MODE_INPUT_OUTPUT);
        gpio_set_pull_mode(i,GPIO_PULLUP_ONLY);
    }
    //REG_CLR_MASK(GPIO_OUT1_DATA,(0xf<<(38-32)));


    //esp32s3
    // gpio_set_direction(35,GPIO_MODE_INPUT_OUTPUT);
    // gpio_set_direction(36,GPIO_MODE_INPUT_OUTPUT);
    // gpio_set_direction(37,GPIO_MODE_INPUT_OUTPUT);
    // gpio_set_direction(38,GPIO_MODE_INPUT_OUTPUT);
    // gpio_set_direction(39,GPIO_MODE_INPUT_OUTPUT);
    // gpio_set_direction(40,GPIO_MODE_INPUT_OUTPUT);
    // gpio_set_direction(41,GPIO_MODE_INPUT_OUTPUT);
    // gpio_set_direction(42,GPIO_MODE_INPUT_OUTPUT);

    gpio_set_direction(1,GPIO_MODE_INPUT);
    gpio_set_direction(8,GPIO_MODE_INPUT);
    value = READ_REG(GPIO_IN_REG);
    int addr = value&0xffff;
    printf("wait ...%08X\n",addr);

    //vPortEnterCritical(&s_init_spinlock);

    #if 0
    while(1){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("%08X\n",READ_REG(GPIO_IN_REG));
    }
    #else
    //测试获得d0~d7的数据
    uint8_t dat[0x40];
    int16_t time[0x40];
    int dat_idx = 0;

    while(1){
        #if 1
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        uint16_t old_d = 0;
        uint16_t d;
        uint32_t t1;
        uint32_t t2;

        for(dat_idx=0;dat_idx<0x40;dat_idx++){
            t1 = _getCycleCount();
            do{
                d = READ_REG(GPIO_IN_REG);
            }while(
                //(d==old_d)
                (0x200&d) /*a15低电平时,读取数据稳定.npn会导致引脚上升.高电平时,npn使gpio9为0*/
            );
            d = d>>1;
            t2 = _getCycleCount();
            do{
                //等待高电平
            }while(!(READ_REG(GPIO_IN_REG)&0x200));
            old_d = d;
            dat[dat_idx] = d;
            time[dat_idx] = t2-t1;
        }
        #if 0
        uint32_t *ptr = (uint32_t*)dat;
        printf(":%08X,%08X,%08X,%08X,%08X,%08X,%08X,%08X,%08X,%08X,%08X,%08X,%08X,%08X,%08X,%08X,\n"
        ":%d,%d,%d,%d,%d,%d,%d,%d,\n"
        ,

        ptr[0+0],ptr[0+1],ptr[0+2],ptr[0+3],
        ptr[4+0],ptr[4+1],ptr[4+2],ptr[4+3],
        ptr[8+0],ptr[8+1],ptr[8+2],ptr[8+3],
        ptr[12+0],ptr[12+1],ptr[12+2],ptr[12+3],
        time[0],time[1],time[2],time[3],time[4],time[5],time[6],time[7]
        );
        #else
        for(i=0;i<4;i++){
            uint8_t *ptr = (uint8_t*)dat + i*0x10;
            printf(":%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,\n",
            ptr[0+0],ptr[0+1],ptr[0+2],ptr[0+3],
            ptr[4+0],ptr[4+1],ptr[4+2],ptr[4+3],
            ptr[8+0],ptr[8+1],ptr[8+2],ptr[8+3],
            ptr[12+0],ptr[12+1],ptr[12+2],ptr[12+3]
            );
        }

        #endif
        #else
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        uint8_t d = READ_REG(GPIO_IN_REG)>>1;
        printf("%02X\n",d);
        #endif
    }
    #endif

    #if 1
    //uint32_t t1 = 0,t2 = 0;
    int t3 = 0;
    while(1){
        value = READ_REG(GPIO_IN1_REG);
        int addr = value;//&0xffff;
        value = rawData[addr];
        value = 0xf;
        printf("wait ...%08X,%d,time:%d,%d\n",addr,i,t2-t1,t3-t2);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        WRITE_REG(GPIO_OUT1_REG,value<<(35-32));

        int ii;
        for(ii=TEST_GPIO_MIN;ii<=TEST_GPIO_MAX;ii++){
            //gpio_set_level(ii,i&1);
            // WRITE_REG(GPIO_OUT_REG,
            //     (READ_REG(GPIO_OUT_REG)&~(1<<8))
            //     |((i&1)<<8)
            // );
        }

        /*

        发出信号t1与接收信号t2的相差时钟为  (WRITE_REG,GPIO_OUT_REG)
        160Mhz:47,3.404255319148936
        240Mhz:57,4.2105263157894735

        //使用WRITE_REG(GPIO_OUT_W1TS_REG,1<<8);
        240Mhz:39,6.153846153846154

        */
        t1 = _getCycleCount();
        #if 1
        WRITE_REG(GPIO_OUT_REG,
                (READ_REG(GPIO_OUT_REG)&~(1<<8))
                |((1)<<8)
        );
        #else
        WRITE_REG(GPIO_OUT_W1TS_REG,1<<8);
        #endif
        while((READ_REG(GPIO_IN_REG)&0x2)==0){
        //直到输入为1
        }
        t2 = _getCycleCount();

        WRITE_REG(GPIO_OUT_REG,
                (READ_REG(GPIO_OUT_REG)&~(1<<8))
                |((0)<<8)
        );

        while((READ_REG(GPIO_IN_REG)&0x2)!=0){
        //直到输入为1
        }
        t3 = _getCycleCount();



        i++;
    }
    #else
    while(1){
        //设置为1
        uint32_t t1 = _getCycleCount();
        WRITE_REG(GPIO_OUT_REG,1<<8);
        // while((READ_REG(GPIO_IN1_REG)&0x4)==0){
        //     //直到输入为1
        // }

        uint32_t t2 = _getCycleCount();


        //设置为0
        // WRITE_REG(GPIO_OUT_REG,
        //     (READ_REG(GPIO_OUT_REG)&~(1<<8))
        //     |((i&1)<<8)
        //  );
        gpio_set_level(8,i&1);
        printf("wait ...%d,c:%d\n",i++,READ_REG(GPIO_IN1_REG));
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }

    #endif


    //taskEXIT_CRITICAL(&s_init_spinlock);

    return;
    #endif

    #if 0
    for (i = 60; i >= 0; i--) {
        t1 = _getCycleCount();
        uint32_t value = 0;
        uint32_t oldvalue = 0;
        uint32_t count = 0;
        do{
            for(x=0;x<0x10;x++){
                //每次执行16次循环
                value = READ_REG(GPIO_IN_REG);
                value = (value&(1<<4))?1:0;
                count += value ^ oldvalue;      //如果不是相同的状态,则+1
                //复制到新值中
                oldvalue = value;
            }
            t2 = _getCycleCount();
        }while((t2-t1)<240*1000*1000);      //1秒

        vTaskDelay(1000 / portTICK_PERIOD_MS);
        int v = gpio_get_level(4);
        printf("Restarting in %d seconds...:%d,%d,%08X,%d\n",i,t2-t1,count,READ_REG(GPIO_IN_REG),v);
    }
    #elif 0
    //测试gpio的反应时间
    for(i=0;i<60;i++){

        #if 1
        if((i&7)==7){
            //触发地址
            t1 = _getCycleCount();
        }
        REG_CLR_MASK(GPIO_OUT_REG,7<<25);
        REG_SET_MASK(GPIO_OUT_REG, (i&7)<<25);
        value = READ_REG(GPIO_IN_REG);
        if((i&7)==7){
            //触发地址
            t1 = _getCycleCount();
            do{
                //t1 = _getCycleCount();
                value = READ_REG(GPIO_IN_REG);
                if(((value>>16)&3)==3){
                    t2 = _getCycleCount();
                    break;
                }
            }while(1);
            printf("get data:%d\n",t2-t1);
        }
        #else
        value = READ_REG(GPIO_IN_REG);
        #endif

        printf("Restarting in %d seconds...,%X\n",i,(value>>16)&3);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    #else
    //测试gpio通用性
    for(i=0;i<5;i++){
        printf("%d waiting\n",i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    //设置gpio
    for(i=0;i<16;i++){
        gpio_reset_pin(i);
        gpio_set_direction(i,GPIO_MODE_INPUT);
        gpio_set_pull_mode(i,GPIO_PULLDOWN_ONLY);
    }

    for(i=35;i<=42;i++){
        gpio_reset_pin(i);
    }

    //esp32s3
    gpio_set_direction(35,GPIO_MODE_OUTPUT);
    gpio_set_direction(36,GPIO_MODE_OUTPUT);
    gpio_set_direction(37,GPIO_MODE_OUTPUT);
    gpio_set_direction(38,GPIO_MODE_OUTPUT);
    gpio_set_direction(39,GPIO_MODE_OUTPUT);
    gpio_set_direction(40,GPIO_MODE_OUTPUT);
    gpio_set_direction(41,GPIO_MODE_OUTPUT);
    gpio_set_direction(42,GPIO_MODE_OUTPUT);
    value = READ_REG(GPIO_IN_REG);

    for(i=0;i<60;i++){
        value = READ_REG(GPIO_IN_REG);
        printf("%d wait,%08X\n",i,value);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    #endif

    #endif //!CONFIG_IDF_TARGET_ESP32C3
}

typedef struct{
    uint8_t scl;
    uint8_t sda;
}ssd1306_drv_t;

static void ssd1306_send(ssd1306_drv_t*drv,uint8_t b);

static void ssd1306_init(ssd1306_drv_t *drv){
    gpio_reset_pin(drv->scl);
    gpio_reset_pin(drv->sda);

    //设置方向
    gpio_set_direction(drv->scl,GPIO_MODE_OUTPUT);
    gpio_set_direction(drv->sda,GPIO_MODE_INPUT_OUTPUT);
}

static void ssd1306_send_start(ssd1306_drv_t*drv){
    gpio_set_level(drv->scl,0);
    //
    gpio_set_level(drv->scl,1);
    gpio_set_level(drv->sda,1);
    /*数据开始时sda由高变低*/
    gpio_set_level(drv->sda,0);
    gpio_set_level(drv->scl,0);
}

static void ssd1306_send_stop(ssd1306_drv_t*drv){
    gpio_set_level(drv->scl,0);
    gpio_set_level(drv->sda,0);
    /*数据停止时sda由低变高*/
    gpio_set_level(drv->scl,1);
    gpio_set_level(drv->sda,1);
}

static int ssd1306_ack(ssd1306_drv_t*drv){
    gpio_set_level(drv->scl,0);

    //
    gpio_set_level(drv->sda,1);
    gpio_set_level(drv->scl,1);
    int ret = gpio_get_level(drv->sda);
    gpio_set_level(drv->scl,0);
    printf("ack:%d\n",ret);

    return ret;
}

static void ssd1306_cmd_start(ssd1306_drv_t*drv,uint8_t cmd){
    ssd1306_send_start(drv);
    ssd1306_send(drv,0x78);     //从机模式
    ssd1306_ack(drv);
    ssd1306_send(drv,0x00);     //写命令
    ssd1306_ack(drv);
    ssd1306_send(drv,cmd);      //写入命令
    ssd1306_ack(drv);
}

static void ssd1306_data_start(ssd1306_drv_t*drv,uint8_t dat){
    ssd1306_send_start(drv);
    ssd1306_send(drv,0x78);     //从机模式
    ssd1306_ack(drv);
    ssd1306_send(drv,0x40);     //写命令
    ssd1306_ack(drv);
    ssd1306_send(drv,dat);      //写入命令
    ssd1306_ack(drv);
}

static void ssd1306_cmd(ssd1306_drv_t*drv,uint8_t cmd){
    ssd1306_cmd_start(drv,cmd);
    ssd1306_send_stop(drv);
}

static void ssd1306_send(ssd1306_drv_t*drv,uint8_t b){
    int i;
    for(i=0;i<8;i++){
        gpio_set_level(drv->sda,(b&0x80)?1:0);
        //低电平
        gpio_set_level(drv->scl,1);

        gpio_set_level(drv->scl,0);
    }
    /*结束*/
    gpio_set_level(drv->sda,1);
    gpio_set_level(drv->scl,1);
    gpio_set_level(drv->scl,0);
}

static int ssd1306_send_and_ack(ssd1306_drv_t*drv,uint8_t b){
    ssd1306_send(drv,b);
    return ssd1306_ack(drv);
}

static void ssd1306_frame(ssd1306_drv_t*drv,uint8_t *pixel /*[8][128]*/){
    ssd1306_send_start(drv);
    ssd1306_send_and_ack(drv,0x21); //设置列起始和结束地址
    ssd1306_send_and_ack(drv,0x00); //列起始地址0
    ssd1306_send_and_ack(drv,0x7f); //结束地址127
    ssd1306_send_and_ack(drv,0x22); //设置页起始地址和结束地址
    ssd1306_send_and_ack(drv,0x00); //页起始地址0
    ssd1306_send_and_ack(drv,0x07); //结束地址7

    int i,j;
    for(i=0;i<128*8;i++){
        ssd1306_send_and_ack(drv,pixel[i]);
    }

    ssd1306_send_stop(drv);
}

const int C_list[0x200] = {
    [24] = 1560,
    [26] = 1500,
    [27] = 1470,
    [28] = 1450,
    [29] = 1420,
    [30] = 1360,
    [46] = 1200,

};

/*获得电阻*/
static int GetR(int mv){
    float v = mv;
    return (int)(v/((3300-v)/100000) + 0.5);
}

/*获得温度*/
static int GetC(int mv,int B){
    if(B==0)B = 3950;   //默认B值为3950
    float b = B;
    float T2 = 273.15+25;
    float Ka = 273.15;
    float Rt1 = GetR(mv);   //热敏电阻值
    float temp = 1.0 / (1.0/T2 + logf(Rt1/100000)/b) - Ka + 0.5;
    return temp;
}

//关闭继电器的最小电压值
static int Off_mV = 1000;
static int On_mV = 1100;
static int relay_pin = 4;

static void test_ssd1306(){
    int i;
    #if 0
#define PIN_SCL 5
#define PIN_SDA 4
    ssd1306_drv_t _drv;
    ssd1306_drv_t *drv = &_drv;
    _drv.scl = PIN_SCL;
    _drv.sda = PIN_SDA;
    ssd1306_init(&_drv);

    //设置主从
    ssd1306_send_start(drv);
    ssd1306_send(drv,0x78);


    //关闭oled显示
    ssd1306_send_and_ack(drv,0xae);

    //初始化
    ssd1306_send_and_ack(drv,0x20); //设置内存寻址模式,水平寻址01
    ssd1306_send_and_ack(drv,0x01);

    ssd1306_send_and_ack(drv,0x81); //设置对比度
    ssd1306_send_and_ack(drv,0xff);

    //开启oled显示
    ssd1306_send_and_ack(drv,0xaf);

    ssd1306_send_stop(drv);


    uint8_t pixel[8][128];

    for(i=0;i<128;i++){
        pixel[2][i] = 0xff;
        memset(pixel,0,sizeof(pixel));
        ssd1306_frame(drv,pixel);
        printf("%d wait\n",i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    #else
#define TEST_LCD_H_RES          (128)
#define TEST_LCD_V_RES          (64)
#if CONFIG_IDF_TARGET_ESP32C3
//c3需要8和9引脚
#define TEST_I2C_SDA_GPIO       (8)
#define TEST_I2C_SCL_GPIO       (9)
#else
#define TEST_I2C_SDA_GPIO       (3)
#define TEST_I2C_SCL_GPIO       (4)
#endif
#define TEST_I2C_HOST_ID        (0)
#define TEST_I2C_DEV_ADDR       (0x3C)
#define TEST_LCD_PIXEL_CLOCK_HZ (400 * 1000)
#define TEST_ESP_OK(n)  n

    gpio_reset_pin(TEST_I2C_SDA_GPIO);
    gpio_reset_pin(TEST_I2C_SCL_GPIO);

    gpio_reset_pin(relay_pin);                          //继电器gpio
    gpio_set_direction(relay_pin,GPIO_MODE_OUTPUT);     //设置输出
    int relay_pin_status = 1;                           //继电器状态
    int relay_pin_status_old = 1;
    gpio_set_level(relay_pin,!relay_pin_status);                        //低电平开启


    #if 0
    const uint8_t pattern[][16] = {{
            0x00, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x00,
            0x00, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x00
        },
        {
            0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x81,
            0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x81
        }
    };
    #endif

    uint8_t pixel[8*128];
    memset(pixel,0x0,sizeof(pixel));
    // pixel[0] = 0x0;
    // pixel[2] = 0x0;
    // pixel[126+0x80*7] = 0x0;

    lcd_pixel_textw_ex(pixel,L"我",1,1,1,128,32,1);

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = TEST_I2C_SDA_GPIO,
        .scl_io_num = TEST_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = TEST_LCD_PIXEL_CLOCK_HZ,
    };

    TEST_ESP_OK(i2c_param_config(TEST_I2C_HOST_ID, &conf));
    TEST_ESP_OK(i2c_driver_install(TEST_I2C_HOST_ID, I2C_MODE_MASTER, 0, 0, 0));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = TEST_I2C_DEV_ADDR,
        .control_phase_bytes = 1, // According to SSD1306 datasheet
        .dc_bit_offset = 6,       // According to SSD1306 datasheet
        .lcd_cmd_bits = 8,        // According to SSD1306 datasheet
        .lcd_param_bits = 8,      // According to SSD1306 datasheet
    };

    TEST_ESP_OK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)TEST_I2C_HOST_ID, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .color_space = ESP_LCD_COLOR_SPACE_MONOCHROME,
        .reset_gpio_num = -1,
    };
    TEST_ESP_OK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));
    TEST_ESP_OK(esp_lcd_panel_reset(panel_handle));
    TEST_ESP_OK(esp_lcd_panel_init(panel_handle));

    #if 0
    for (int i = 0; i < TEST_LCD_H_RES / 16; i++) {
        for (int j = 0; j < TEST_LCD_V_RES / 8; j++) {
            TEST_ESP_OK(esp_lcd_panel_draw_bitmap(panel_handle, i * 16, j * 8, i * 16 + 16, j * 8 + 8, pattern[i & 0x01]));
        }
    }
    #else
    TEST_ESP_OK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 128, 64, pixel));
    // for (int i = 0; i < TEST_LCD_H_RES / 16; i++) {
    //     for (int j = 0; j < TEST_LCD_V_RES / 8; j++) {
    //         TEST_ESP_OK(esp_lcd_panel_draw_bitmap(panel_handle, i * 16, j * 8, i * 16 + 16, j * 8 + 8, pattern[i & 0x01]));
    //     }
    // }
    #endif



    #endif

    /*adc采集*/


    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL_3,ADC_ATTEN_11db);
    adc1_config_channel_atten(ADC_CHANNEL_2,ADC_ATTEN_11db);

    esp_adc_cal_characteristics_t *adc_chars;
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1,
        ADC_ATTEN_11db, ADC_WIDTH_BIT_12, 1100, adc_chars);

    int j;
    for(i=0;i<60*3;i++){
        /*每3分钟重启一次*/

        //获得10次adc的值
        int max_v = 0,min_v = 0xffff;
        int max_idx = -1,min_idx = -1;
        int max2_v = 0,min2_v = 0xffff;
        int max2_idx = -1,min2_idx = -1;
        int adc_values[10];
        int adc_values2[10];
        for(j=0;j<10;j++){
            int value = adc1_get_raw(ADC_CHANNEL_3);
            adc_values[j] = value;
            /*处理最小和最大值*/
            if(max_v<value){
                max_idx = j;
                max_v = value;
            }
            if(min_v>value){
                min_idx = j;
                min_v = value;
            }

            value = adc1_get_raw(ADC_CHANNEL_2);
            adc_values2[j] = value;
            /*处理最小和最大值*/
            if(max2_v<value){
                max2_idx = j;
                max2_v = value;
            }
            if(min2_v>value){
                min2_idx = j;
                min2_v = value;
            }


            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        //计算平均值
        int value = 0;
        for(j=0;j<10;j++){
            if(min_idx==j||max_idx==j){
                /*不处理*/
            }
            else{
                value += adc_values[j];
            }
        }
        if(max_idx==min_idx){
            value /= 9; //要处理9
        }
        else{
            //去掉两个
            value /= 8;
        }

        int value2 = 0;
        for(j=0;j<10;j++){
            if(min2_idx==j||max2_idx==j){
                /*不处理*/
            }
            else{
                value2 += adc_values2[j];
            }
        }
        if(max2_idx==min2_idx){
            value2 /= 9; //要处理9
        }
        else{
            //去掉两个
            value2 /= 8;
        }



        /*电压值*/
        float v = (float)value * 2500.0 / 4095.0;
        float Va = 3300;        //总电压值
        float Vb = 3300 - v;    //100K欧姆的电压值


        int mV = esp_adc_cal_raw_to_voltage(value,adc_chars);
        int mV2 = esp_adc_cal_raw_to_voltage(value2,adc_chars);

        int C = GetC(mV,0); //获得温度

        #if 0
        if(mV<Off_mV){
            relay_pin_status = 0;   //关闭加热
        }
        else if(mV>On_mV){
            relay_pin_status = 1;   //开启加热
        }
        #else
        //使用摄氏度来调温
        int C2 = mV2/5;
        if(C>(C2-10)){
            //由于断电后，会继续加热,所以需要提前断电
            //关闭加热
            relay_pin_status = 0;   //关闭加热
        }
        else if(C<C2-10){
            //相差10摄氏度的误差
            relay_pin_status = 1;   //开启加热
        }
        #endif

        /*设置继电器*/
        if(relay_pin_status!=relay_pin_status_old){
            //已经更改继电器状态
            relay_pin_status_old = relay_pin_status;
            gpio_set_level(relay_pin,!relay_pin_status);    //低电平开启
        }


        wchar_t text[0x40];
        memset(text,0,sizeof(text));
        swprintf(text,1023,L"R:%d,%dmV",value,mV);
        memset(pixel,0,sizeof(pixel));
        lcd_pixel_textw_ex(pixel,text,1,1,1,128,32,1);
        memset(text,0,sizeof(text));
        swprintf(text,1023,relay_pin_status?L"加热:%d=>%d":L"保温:%d=>%d",C,mV2/5);

        lcd_pixel_textw_ex(pixel,text,1,1,16,128,32,1);
        TEST_ESP_OK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 128, 64, pixel));


        printf("%d wait,%d,%dmV,%dmV\n",i,value,mV,mV2);

    }

    TEST_ESP_OK(esp_lcd_panel_del(panel_handle));
    TEST_ESP_OK(esp_lcd_panel_io_del(io_handle));
    TEST_ESP_OK(i2c_driver_delete(TEST_I2C_HOST_ID));
}

void test_cycles(){
    uint8_t data[0x10];
    uint32_t addr = &switch_addr;
    uint32_t addr2 = GPIO_IN_REG;
    memset(data,3,sizeof(data));
    int i = 0;
    for(i=0;i<=21;i++){
        gpio_reset_pin(i);
        gpio_set_direction(i,GPIO_MODE_INPUT);
    }

    // for(i=26;i<32;i++){
    //     gpio_reset_pin(i);
    //     gpio_set_direction(i,GPIO_MODE_INPUT);
    // }

    while(1){
        uint32_t x = 0;
        int t1 = _getCycleCount();
        //_getCycleCount 需要额外的指令,所以. 每条指令需要-1 得到单指令周期

        //asm volatile("l32i.n %0,%1,0" : : "r"(x),"r"(data):);       //2:cycles
        //asm volatile("l32i.n %0,%1,1" : : "r"(x),"r"(data):);       //5:cycles
        //asm volatile("l32i.n %0,%1,2" : : "r"(x),"r"(data):);       //5:cycles
        //asm volatile("l32i.n %0,%1,4" : : "r"(x),"r"(data):);       //2:cycles

        // asm volatile("l8ui %0,%1,0" : : "r"(x),"r"(data):);       //2:cycles
        // asm volatile("l8ui %0,%1,1" : : "r"(x),"r"(data):);       //2:cycles
        // asm volatile("l8ui %0,%1,2" : : "r"(x),"r"(data):);       //2:cycles
        // asm volatile("l8ui %0,%1,4" : : "r"(x),"r"(data):);       //2:cycles

        // asm volatile("extui	%0, %1, 0, 14" : : "r"(x),"r"(data):);       //2:cycles

        {
            asm volatile("memw\n");                                 //+1
            //asm volatile("l8ui %0,%1,0" : : "r"(x),"r"(addr):);     //+1
            asm volatile("l8ui %0,%1,0" : : "r"(x),"r"(addr2):);     //+15
            //asm volatile("l32i.n %0,%1,0" : : "r"(x),"r"(addr2):);     //+15
            /*10 cycles*/
            asm volatile("extui	%0, %1, 9, 8\n" : : "r"(x),"r"(data): );
            asm volatile("slli	%0, %1, 8\n"     : : "r"(x),"r"(data): );
            asm volatile("add.n	%0, %1, a3\n"    : : "r"(x),"r"(data): );
            asm volatile("extui	%0, %1, 0, 16\n" : : "r"(x),"r"(data): );
            asm volatile("srli	%0, %1, 14\n"   : : "r"(x),"r"(data): );
            asm volatile("slli	%0, %1, 2\n"   : : "r"(x),"r"(data): );
            asm volatile("add.n	%0, %1, %0\n"  : : "r"(x),"r"(data): );
            asm volatile("extui	%0, %1, 0, 14\n" : : "r"(x),"r"(data): );
            asm volatile("add.n	%0, %1, %0\n"  : : "r"(x),"r"(data): );
        }



        int t2 = _getCycleCount();
        printf("TT:%d,%02X\n",t2-t1,x);
        Delay(1000);
    }
}

void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
    //test_gpio();

    //test_74hc595();

    //测试时钟
    //test_clk();

    //测试nes
    //test_nes();

    //测试指令周期
    //test_cycles();

    //测试phi2
    //test_phi2();

    #if 1
    //测试模拟sram
    xTaskCreatePinnedToCore(&core2_task, "core2 task",
                                        ESP_TASK_MAIN_STACK, NULL,
                                        ESP_TASK_MAIN_PRIO, NULL, 1);
    test_sram();
    #endif

    //测试sram读写
    test_sram_rw();

    //测试oled
    //test_ssd1306();

    //gpio_testAll();
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
