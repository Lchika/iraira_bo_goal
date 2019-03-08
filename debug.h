/*
 * デバッグ用Serial出力
 * 参考:http://monakaice88.hatenablog.com/entry/20141227/1419634072
 */

#ifndef __DEBUG_H__
#define __DEBUG_H__

//  デバッグ出力ON/OFF用マクロ
#define DEBUG

#ifdef DEBUG
    #define BeginDebugPrint()    Serial.begin( 9600 )
    #define DebugPrint( message )\
        {\
            char __buff__[ 128 ];\
            sprintf( __buff__\
                   , "%s (Func:%s)"\
                   , message\
                   , __func__ );\
            Serial.println( __buff__ );\
            Serial.flush();\
        }
#else
    #define BeginDebugPrint()
    #define DebugPrint( message )
#endif // DEBUG
#endif // __DEBUG_H__
