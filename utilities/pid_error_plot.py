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
        view = pg.PlotWidget(title='UR5e PID Error')    # The actual window
        curve = view.plot(pen=pg.mkPen('r', width=2), symbol='o')      # Initialize the plotting "pen" object
        view.show()

        data = []

        def update_data():
            while self.pipe.poll():     # self.pipe.poll() returns True if there is a message waiting.
                data.append(self.pipe.recv())   # recv() pulls the data out of the buffer.
                if len(data) > 500: data.pop(0)
        
            curve.setData(data)     # Redraw the entire curve with the new dataset every cycle

        timer = QtCore.QTimer()     # Create a timer object
        timer.timeout.connect(update_data)      # When the timer elapses, run the connected function.
        timer.start(16)     # Set the update interval to 16ms

        # Starts Qt event loop
        app.exec()