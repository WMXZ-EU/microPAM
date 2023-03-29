
enum STATUS
{
    STOPPED=-1,
    CLOSED=0,
    OPENED=1,
    RUNNING=2,
    DOCLOSE=3,
    MUSTSTOP=4,
    //
    MUST_REBOOT=-2
};

int16_t filing_init(void);

int16_t saveData(int16_t status);

extern uint32_t logBuffer[];
