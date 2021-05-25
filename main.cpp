#include "pipeline.h"


int main(){

    pipeline pipe;
    pipe.display_menu();
    pipe.load_data();

    pipe.start();
    
    pipe.showCloud();

    pipe.showInfo();

    return 0;
}
