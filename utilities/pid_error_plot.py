import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
import numpy as np

import multiprocessing as mp


class ErrorPlot(mp.Process):
    def __init__(self, pipe):
        super().__init__()
        self.pipe = pipe

    def run(self):
        """
        run() is a magic method in multiprocessing; you must use that name. When process.start() is called in the
        main script, it internally looks for this run() method. Never call run() directly.
        """
        # Set up the plot GUI
        app = QtWidgets.QApplication([])    # The high level manager
        multi_plot_window = pg.GraphicsLayoutWidget(show=True, size=(960, 540), title="UR5e PID Twist Errors")

        w_curves = self.create_multi_line_plot(
            multi_plot_window,
            (0, 0),
            'Twist rotational error',
            3,
            (0, 500),
            (-0.0025, 0.0025),
            ('Iteration', 'Error (radians)')
        )

        v_curves = self.create_multi_line_plot(
            multi_plot_window,
            (0, 1),
            'Twist linear error',
            3,
            (0, 500),
            (-1500, 1500),
            ('Iteration', 'Error (mm)')
        )

        data = [[], [], [], [], [], []]

        def update_data():
            while self.pipe.poll():     # self.pipe.poll() returns True if there is a message waiting.
                full_data = self.pipe.recv()    # recv() pulls the data out of the buffer.

                # Create plot for twist rotational eror
                for i in range(6):
                    data[i].append(full_data[i])
                    if len(data[i]) > 500: data[i].pop(0)

                for i in range(3):
                    w_curves[i - 3].setData(data[i])
                
                for i in range(3, 6):
                    v_curves[i - 3].setData(data[i])

        timer = QtCore.QTimer()     # Create a timer object
        timer.timeout.connect(update_data)      # When the timer elapses, run the connected function.
        timer.start(16)     # Set the update interval to 16ms

        # Starts Qt event loop
        app.exec()

    def create_multi_line_plot(
            self,
            layout_widget: pg.GraphicsLayoutWidget,
            position: tuple[int],
            title: str,
            num_curves: int,
            x_range: tuple[float],
            y_range: tuple[float],
            axis_labels: tuple[str]
    ):
        
        plot = layout_widget.addPlot(row=position[0], col=position[1], title=title)
        plot.addLegend()
        plot.setXRange(x_range[0], x_range[1])
        plot.setYRange(y_range[0], y_range[1])
        plot.setLabel(axis='left', text=axis_labels[1])
        plot.setLabel(axis='bottom', text=axis_labels[0])

        curves = []
        colours = ['r', 'g', 'b']   # Match Webots' xyz colour convention
        axes = ['x', 'y', 'z']

        for i in range(num_curves):
            curves.append(plot.plot(pen=pg.mkPen(colours[i], width=2), name=axes[i]))

        return curves