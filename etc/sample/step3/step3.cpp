/* step3 switch���ƃL�[�����֐��@*/
#include "dm3.h"
#include <time.h>
#include <stdlib.h>

double R[12] ={1,0,0,0, 0,1,0,0, 0,0,1,0};  // ��]�s�񂪊i�[�����z��, �ʒu(x,y,z)[m]
double p[3] = {0.0, 0.0, 0.05};   // �ʒu(x,y,z)[m]
double sides[3] = {0.1, 0.1, 0.1}; // �����̂̃T�C�Y(x, y, z)[m]

double start_x = 0.0, start_y = 0.05, start_z = 0.0; // �����ʒu

void command(int cmd)
{
    float xyz;

    switch (cmd)
    {
    case 'z':
        start_z += 0.1;
        break;
    default:
        printf("Input z key \n");
    }
}


void simLoop(int pause)        /***  �V�~�����[�V�������[�v�@***/
{
    int i, j, num = 10; // �����̂̐�
    float red = 0.0, green = 0.0, blue = 0.0; // �ԁC�΁C����

    for (i = 0; i < num; i++)
    {
        for (j = 0; j < num; j++)
        {
            red = (float) rand()/RAND_MAX; // �Ԑ����𗐐��Ō���
            dsSetColor(red, green, blue); // �F�̐ݒ�
            p[0] = start_x; // �ʒu��x����
            p[1] = i * 0.1 + start_y; // �ʒu��y����
            p[2] = j * 0.1 + 0.05 + start_z; // �ʒu��z����
            dsDrawBox(p,R,sides); // �����̂̕\��
        }
    }
}

int main()         /*** main�֐� ***/
{
    srand(time(NULL)); // �����̏�����
    dmLoop(800, 600, simLoop, command); // �V�~�����[�V�������[�v �E�C���h�E�̕��C��
    return 0;
}
