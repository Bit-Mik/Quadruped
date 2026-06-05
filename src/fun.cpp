#include "fun.h"
#include "globals.h"
#include <servo_manual.h>
#include <config.h>

void danceBounce()
{
    for(int i=0;i<10;i++)
    {
        setLegPosition(0,0,0,-17.5);
        setLegPosition(1,0,0,-17.5);
        setLegPosition(2,0,0,-17.5);
        setLegPosition(3,0,0,-17.5);
        delay(150);

        setLegPosition(0,0,0,-19);
        setLegPosition(1,0,0,-19);
        setLegPosition(2,0,0,-19);
        setLegPosition(3,0,0,-19);
        delay(150);
    }
}

void danceSway()
{
    for(int i=0;i<4;i++)
    {
        setLegPosition(LEG_FR,0, 2.1,-18);
        setLegPosition(LEG_BR,0, 2.1,-18);

        setLegPosition(LEG_FL,0,-2.1,-18);
        setLegPosition(LEG_BL,0,-2.1,-18);

        delay(300);

        setLegPosition(LEG_FR,0,-2.1,-18);
        setLegPosition(LEG_BR,0,-2.1,-18);

        setLegPosition(LEG_FL,0, 2.1,-18);
        setLegPosition(LEG_BL,0, 2.1,-18);

        delay(300);
    }
}

void happyDance()
{
    for(int i=0;i<5;i++)
    {
        setLegPosition(LEG_FR,0,0,-16);
        delay(200);

        setLegPosition(LEG_FR,0,0,-19);
        delay(100);

        setLegPosition(LEG_FL,0,0,-16);
        delay(200);

        setLegPosition(LEG_FL,0,0,-19);
        delay(100);
    }
}

void twerk()
{
    for(int i=0;i<10;i++)
    {
        setLegPosition(LEG_BR, 2,0,-18);
        setLegPosition(LEG_BL, 2,0,-18);
        delay(120);

        setLegPosition(LEG_BR,-2,0,-18);
        setLegPosition(LEG_BL,-2,0,-18);
        delay(120);
    }
}

void headBang()
{
    for(int i=0;i<8;i++)
    {
        setLegPosition(LEG_FR,0,0,-17);
        setLegPosition(LEG_FL,0,0,-17);

        setLegPosition(LEG_BR,0,0,-19);
        setLegPosition(LEG_BL,0,0,-19);

        delay(500);

        setLegPosition(LEG_FR,0,0,-19);
        setLegPosition(LEG_FL,0,0,-19);

        setLegPosition(LEG_BR,0,0,-17);
        setLegPosition(LEG_BL,0,0,-17);

        delay(500);
    }
}