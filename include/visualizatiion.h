#ifndef _VISUALIZE_H_
#define _VISUALIZE_H_
#include <sciplot/sciplot.hpp>


class Visualize{
    public:
        // Initialize the constructor
        Visualize(int x_max, int y_max): xmax(x_max), ymax(y_max){}
        void display_graph(sciplot::Vec& x, int amplitude, int y_offset, bool display=false);
        
        // Destructor
        ~Visualize();

    private:
        int xmax;
        int ymax;
};
#endif /*_VISUALIZE_H_*/