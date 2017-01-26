#include "PT_VisualAttention.h"

//======================================================================================
// Otsu Threshold�� �ξ������� ������ ���� 
// �� �������� ������ �� ���������� variance�� ���� �۰� �ϴ� ���� �̿��Ͽ� 
// �� ������ threshold�� �Ͽ���.   
//======================================================================================
unsigned char * Otsu_option(unsigned char *blockData, int subwidth, int subheight, int option)
{ 
    int i, j, k, m, value, i_th, thresh=0;
    
    #define graylevel 256
    unsigned char *outData;
    int level_histo[graylevel]={0,}, sum_h, sum_l, sum1, sum2;
    double p[graylevel]={0,}, min_J=100000, var_w=0, var_b=0, var_1=0, var_2=0, mean_1=0, mean_2=0, size_ratio_1=0, size_ratio_2=0, J_value;

    //==================================================================================
    //        Implementation of histogram
    //   �ѱ���� �׳� ������׷��� ����� ���̴�. 
    //    �׸��� ������׷��� Ȯ���� ����� �� ���̴�.(Ȯ���� ������׷���/��ü�ȼ���) 
    //=================================================================================

    int start, b_height, b_width;
    if(!option)
    { // �Ϲ����� ���..
        start=0; b_height = subheight;
        b_width = subwidth;
    }
    else
    {
        // �̰��� �� �׵θ��� �����ϰ� �ϰ� ���� ���..
        start=option; b_height = subheight-option;
        b_width = subwidth-option;
    }

    for(i=start; i< b_height; i++) 
    {
        for(j=start; j<b_width; j++)
        {
            value = (int)blockData[i*subwidth+j]; 
            level_histo[value]++;
            p[value]=(double)level_histo[value]/(double)(b_height*b_height);

        }
    }
    //=================================================================================
    //         Output space allocate
    //   �׳� �޸� ��� ��
    //=================================================================================
    if ((outData = (unsigned char *)malloc(sizeof(char)*subheight*subwidth))==NULL) 
    {
        printf("Not enough memory to allocate buffer\n"); 
        exit(1); 
    } 

    for(i=0; i<subheight*subwidth; i++)
    {
        outData[i]=100;
    }

    //===============================================================================
    //                   Otsu�� ���� ������ �� ���̴�.     
    //===============================================================================

    for (i_th=0; i_th<graylevel-1; i_th++)
    {
        sum_h=0;
        sum_l=0;
        size_ratio_1=0;
        size_ratio_2=0;

         for (j=0; j<graylevel; j++)
        {
            if(j>i_th)
            {
                sum_h=sum_h+level_histo[j];
                size_ratio_2=size_ratio_2+p[j];
            }            
            else
            {
                sum_l=sum_l+level_histo[j];
                size_ratio_1=size_ratio_1+p[j];
            }
        }
        sum1=0;
        sum2=0;
        
        if (sum_l!=0)
        {
            for (k=0; k<graylevel; k++)
            {
                if(k<i_th)
                    sum1=sum1+k*level_histo[k];
                else
                    sum2=sum2+k*level_histo[k];
            }
            mean_1=(double)sum1/(double)sum_l;
            mean_2=(double)sum2/(double)sum_h;

            var_1=0;
            var_2=0;

            for (m=0; m<graylevel; m++)
            {
                if(m<i_th)
                    var_1=var_1+((double)m-mean_1)*((double)m-mean_1)*p[m]/size_ratio_1;
                else
                    var_2=var_2+((double)m-mean_2)*((double)m-mean_2)*p[m]/size_ratio_2;
            }

            var_w=size_ratio_1*var_1+size_ratio_2*var_2;
            var_b=size_ratio_1*size_ratio_2*(mean_1-mean_2)*(mean_1-mean_2);

        //    J_value=fabs(var_w/var_b);
            J_value=double(var_w/var_b);

            if (min_J>J_value)
            {
                min_J=J_value;
                thresh=i_th;
            }
        }
    }
//    printf("threshold = %d\n", thresh);
    //===========================================================================
    //       threshold�� ������ ������ϱ� ���� �ؾ���
    //===========================================================================
            
    for(i=start; i<b_height; i++)
    {
        for(j=start; j<b_width; j++) 
        {
            if (blockData[i*subwidth+j]<thresh+25)
                outData[i*subwidth+j]=0; 
            else
                outData[i*subwidth+j]=255;
        }
    }            

    return outData;
}
