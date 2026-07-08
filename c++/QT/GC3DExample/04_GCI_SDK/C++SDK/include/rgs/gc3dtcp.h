#pragma once
#ifndef GC3DTCP_H
#define GC3DTCP_H
#include <string>
#include <WinSock2.h>
#include "rgsdef.h"

typedef void(*CallbackFunTcp)(void *);

#define MAX_CHARBUFFER 1024

//错误列表
enum TCP_ERROR
{
    TCP_SUCCESS = 0,
    TCP_VISIBLE_FALSE,//未启用
    TCP_TYPE_ERROR,//类型错误
    TCP_BIND_FAILED,//绑定失败
    TCP_LISTEN_FAILED,//监听失败
    TCP_CONNECT_FAILED,//链接失败
    TCP_OUTOFRANGE,//字符超出限制

};

//TCP 连接类型
enum TCP_CONNECT_TYPE {
    TCP_CLIENT=0,
    TCP_SERVER
};

namespace gc3d {
class __declspec(dllexport) GC3DTCP
{
public:
    GC3DTCP();
    ~GC3DTCP();
public:
    static GC3DTCP *GetTCP();
    //设置tcp通讯1:客户端；0：服务端
    int TCP_SetVisible(TCP_CONNECT_TYPE flags = TCP_CLIENT);
    //设置超时t单位ms
    int SetTimeOut(int t);
    //打开服务端
    int TCP_OpenServer(int Port);
    //链接服务端
    int TCP_Connect(const char *Ip, int Port);
    //发送消息
    int TCP_SendMsg(const char *msg, int size);
    //接收消息（自定义接收）
    int TCP_RecvMsg(char *msg);
    //获取线程中的消息（读取接收线程中的消息）
    int TCP_SetCallBackFun(CallbackFunTcp fun);
    //关闭链接
    int TCP_Close();
    //错误名称
    std::string GetErrorStr(TCP_ERROR type);
protected:
    //TCP线程
    void ReceiveMsg();//在这里添加自定义协议
    void WaitForClient();//
    static DWORD WINAPI ThreadTCP(LPVOID lpThreadParameter);
    static DWORD WINAPI TCPAccept(LPVOID lpThreadParameter);
protected:
    /************************TCP*********************/
    bool        m_UseTcp;
    WSADATA     m_WsaData;
    SOCKET      m_nSock;
    SOCKET      m_ClientSock;
    sockaddr_in m_SockAddr;
    TCP_CONNECT_TYPE    m_nType;//true:服务端;false:客户端
    bool        m_Choose;//选择由类接收或者自己接收
    char        m_LastMsg[MAXBYTE];
    char        m_MsgBuffer[MAXBYTE];
    int         m_TimeOut;
    CallbackFunTcp m_Fun;
};

class __declspec(dllexport) GTCPModbus {
public:
    GTCPModbus();
    ~GTCPModbus();
    int SetTimeOut(int t);
    //链接服务端
    int TCP_Modbus_Connect(const char *Ip, int Port=502);
    //断开服务器
    void TCP_Modbus_Unconnect();
    //发送消息
    int TCP_Modbus_SendMsg(int startAddress, const uint16_t *wdata, int size);
    //接收消息（自定义接收）
    int TCP_Modbus_RecvMsg(int startAddress, uint16_t *wdata, int size);
};


}



#endif // GC3DCP_H
