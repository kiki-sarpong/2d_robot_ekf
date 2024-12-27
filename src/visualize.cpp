#include <sciplot/sciplot.hpp>
#include <glog/logging.h>
#include <iostream>
#include "visualizatiion.h"

void Visualize::display_graph(sciplot::Vec& x, int amplitude, int y_offset, bool display){
    LOG(INFO) << "display flag is set to:" << display;
    if (display){
        LOG(INFO) << "Starting display...";
        
        // Create a Plot object
        sciplot::Plot2D plot;
        // plot.fontName("Georgia");
        // plot.fontSize(2);

        // Set the x and y labels
        // plot.xlabel("x");//.legend().fontSize(font_size).fontName(font_name);
        // plot.ylabel("y");//.legend().fontSize(font_size).fontName(font_name);

        // Set the x and y ranges
        plot.xrange(0.0, xmax);//.legend().fontSize(font_size).fontName(font_name);
        plot.yrange(0.0, ymax);//.legend().fontSize(font_size).fontName(font_name);

        // // Set the legend to be on the bottom along the horizontal
        // plot.legend()
        //     .atOutsideBottom()
        //     .displayHorizontal()
        //     .displayExpandWidthBy(1);
        //     // .fontSize(font_size)
        //     // .fontName(font_name);

        plot.drawCurveWithPoints(x, amplitude * std::sin(x) + y_offset)
            .pointSize(1);//.label("sin(x)");

        // Create figure to hold plot
        sciplot::Figure fig = {{plot}};
        // Create canvas to hold figure
        sciplot::Canvas canvas = {{fig}};

        // Show the plot in a pop-up window
        canvas.show();

        // Save the plot to a PDF file
        // canvas.save("example-sine-functions.pdf");
        LOG(INFO) << "Ending display.";
        return;
    }
    LOG(INFO) << "No display to be shown.";

}

// Explicitly define the destructor
Visualize::~Visualize() {
    // Custom cleanup (if needed)
    // Example: No dynamic memory here, but you could delete resources if required
    std::cout << "Destructor called" << std::endl;
}