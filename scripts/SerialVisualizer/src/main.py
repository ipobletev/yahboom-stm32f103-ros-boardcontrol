import sys
import os
from PySide6.QtWidgets import QApplication

# Add current directory to path to allow absolute imports within the package
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from gui.main_window import SerialVisualizerWindow

def main():
    app = QApplication(sys.argv)
    window = SerialVisualizerWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
