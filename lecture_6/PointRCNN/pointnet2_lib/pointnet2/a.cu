#include <stdio.h>
#include <stdlib.h>
#include <cuda_runtime.h>

//初始化CUDA
int count=0;

bool InitCUDA()
{
    printf("Start to detecte devices.........\n");//显示检测到的设备数

    cudaGetDeviceCount(&count);//检测计算能力大于等于1.0 的设备数

    if(count == 0)
    {
        fprintf(stderr, "There is no device.\n");
        return false;
    }

    printf("%d device/s detected.\n",count);//显示检测到的设备数

    int i;
    for(i = 0; i < count; i++)
    {//依次验证检测到的设备是否支持CUDA
        cudaDeviceProp prop;
        if(cudaGetDeviceProperties(&prop, i) == cudaSuccess) 
        {//获得设备属性并验证是否正确
            if(prop.major >= 1)//验证主计算能力，即计算能力的第一位数是否大于1
            {
                printf("Device %d: %s supportsCUDA %d.%d.\n",i+1,prop.name,prop.major,prop.minor);//显示检测到的设备支持的CUDA 版本
                break;
            }
        }
    }

    if(i == count) 
    {//没有支持CUDA1.x 的设备
        fprintf(stderr, "There is no device supporting CUDA 1.x.\n");
        return false;
    }

    cudaSetDevice(i);//设置设备为主叫线程的当前设备
    return true;
}

int main()
{
    if(!InitCUDA()) 
    {//初始化失败返回系统int argc, char** argv
        return 0;
    }

    printf("Hello GPU! CUDA has been initialized.\n");

    //exit(argc ? EXIT_SUCCESS : EXIT_FAILURE);
    return 0;//返回系统
}
