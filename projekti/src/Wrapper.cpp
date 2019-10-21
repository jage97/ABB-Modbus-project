/*
 * Wrapper.cpp
 *
 *  Created on: 23.1.2019
 *      Author: Jaakk
 */

#include "Wrapper.h"
#include "ITM_write.h"
#include "board.h"
#include "chip.h"
#include <string>
#include <stdio.h>

Wrapper::Wrapper() {
        ITM_init();
        // TODO Auto-generated constructor stub
}
void Wrapper::print(int value, int value2){
        std::string k ="a = "+std::to_string(value2)+"  button = "+ std::to_string(value) +"\n";
        char p[100] = {};
        for(int i=0;i<k.size();i++){
                p[i] = k[i];
        }
        ITM_write(p);
}
void Wrapper::print2(int value){
        std::string k =std::to_string(value) +" \n";
        char p[100] = {};
        for(int i=0;i<k.size();i++){
                p[i] = k[i];
        }
        ITM_write(p);
}

Wrapper::~Wrapper() {
        // TODO Auto-generated destructor stub
}
