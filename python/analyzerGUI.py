import sys
import os
import datetime
import matplotlib
# Set the backend to QtAgg
matplotlib.use('QtAgg')

from PySide2.QtWidgets import QApplication, QWidget, QVBoxLayout, QAbstractItemView, QPushButton, QListWidget, QFileDialog, QMessageBox
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import matplotlib.pyplot as plt

class DataPlotter(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Signal Plotter')
        self.setGeometry(100, 100, 800, 600)

        # Layouts
        main_layout = QVBoxLayout(self)
        button_layout = QVBoxLayout()

        # Button to select folder
        self.select_folder_btn = QPushButton("Select Folder", self)
        self.select_folder_btn.clicked.connect(self.select_folder)
        button_layout.addWidget(self.select_folder_btn)

        # List box to display .txt files
        self.file_list = QListWidget(self)
        button_layout.addWidget(self.file_list)

        # Button to load selected file
        self.load_file_btn = QPushButton("Load Selected File", self)
        self.load_file_btn.clicked.connect(self.load_file)
        button_layout.addWidget(self.load_file_btn)

        # List box to display signals
        self.signal_list = QListWidget(self)
        button_layout.addWidget(self.signal_list)
        self.signal_list.setSelectionMode(QAbstractItemView.SelectionMode.MultiSelection)

        # Button to plot selected signals
        self.plot_signals_btn = QPushButton("Plot Selected Signals", self)
        self.plot_signals_btn.clicked.connect(self.plot_signals)
        button_layout.addWidget(self.plot_signals_btn)

        # Matplotlib Figure and Canvas
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)

        # Add a toolbar for zooming and panning
        self.toolbar = NavigationToolbar(self.canvas, self)

        # Add widgets to the layout
        main_layout.addLayout(button_layout)
        main_layout.addWidget(self.toolbar)  # Add the toolbar
        main_layout.addWidget(self.canvas)   # Add the Matplotlib canvas

    def select_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "Select Folder")
        if folder:
            self.file_list.clear()
            txt_files = [f for f in os.listdir(folder) if f.endswith('.txt')]
            for file in txt_files:
                self.file_list.addItem(os.path.join(folder, file))

    def load_file(self):
        selected_file = self.file_list.currentItem()
        if selected_file:
            file_path = selected_file.text()
            try:
                with open(file_path, 'r') as file:
                    self.lines = file.readlines()

                if len(self.lines) > 0:
                    headers = self.lines[0].strip().split(';')[1:]  # Skip the first 'time' header
                    self.signal_list.clear()
                    for header in headers:
                        self.signal_list.addItem(header)

                    self.times = []  # Initialize self.times
                    self.signal_data = {signal: [] for signal in headers}  # Initialize signal_data correctly
                    
                    for line in self.lines[1:]:
                        parts = line.strip().split(';')
                        time_str = parts[0]
                        time_obj = datetime.datetime.strptime(time_str, "%Y-%m-%d-%H:%M:%S.%f")
                        self.times.append(time_obj)
                        
                        for i, signal in enumerate(headers):
                            self.signal_data[signal].append(float(parts[i + 1]))  # Use index correctly
                else:
                    QMessageBox.warning(self, "Warning", "The selected file is empty.")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load file: {e}")
        else:
            QMessageBox.warning(self, "Warning", "No file selected")

    def plot_signals(self):
        selected_signals = [item.text() for item in self.signal_list.selectedItems()]
        if selected_signals:
            self.figure.clear()
            ax = self.figure.add_subplot(111)

            for signal in selected_signals:
                ax.plot(self.times, self.signal_data[signal], label=signal)

            ax.set_xlabel("Time")
            ax.set_ylabel("Signal Value")
            ax.legend()
            self.canvas.draw()
        else:
            QMessageBox.warning(self, "Warning", "No signals selected")


def main():
    app = QApplication(sys.argv)
    main_window = DataPlotter()
    main_window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
