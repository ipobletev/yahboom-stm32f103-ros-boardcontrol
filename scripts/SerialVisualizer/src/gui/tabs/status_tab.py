from PySide6.QtWidgets import QWidget, QVBoxLayout, QGroupBox, QTableWidget, QTableWidgetItem, QHeaderView
from PySide6.QtCore import Qt

class StatusTab(QWidget):
    def __init__(self, sys_errors):
        super().__init__()
        self.sys_errors = sys_errors
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout(self)

        # Errors Table
        err_detailed_group = QGroupBox("Health Monitoring & Error Codes")
        err_detailed_layout = QVBoxLayout(err_detailed_group)
        self.error_table = QTableWidget(len(self.sys_errors), 3)
        self.error_table.setHorizontalHeaderLabels(["Component / System", "Error Code", "Status"])
        self.error_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.error_table.verticalHeader().setVisible(False)
        self.error_table.setEditTriggers(QTableWidget.NoEditTriggers)
        
        # Pre-fill table with "-" (No data yet)
        for row, (bit, name) in enumerate(self.sys_errors.items()):
            self.error_table.setItem(row, 0, QTableWidgetItem(name))
            
            # Error Code Column (Bitmask)
            item_code = QTableWidgetItem(f"0x{bit:08X}")
            item_code.setTextAlignment(Qt.AlignCenter)
            item_code.setForeground(Qt.gray)
            self.error_table.setItem(row, 1, item_code)

            # Status Column
            item_status = QTableWidgetItem("-")
            item_status.setTextAlignment(Qt.AlignCenter)
            item_status.setForeground(Qt.gray)
            self.error_table.setItem(row, 2, item_status)
            
        err_detailed_layout.addWidget(self.error_table)
        layout.addWidget(err_detailed_group)
        layout.addStretch()

    def update_errors(self, error_code):
        for row, (bit, name) in enumerate(self.sys_errors.items()):
            is_active = (error_code & bit) != 0
            code_item = self.error_table.item(row, 1)
            status_item = self.error_table.item(row, 2)
            
            if is_active:
                status_item.setText("FAULT")
                status_item.setBackground(Qt.red)
                status_item.setForeground(Qt.white)
                code_item.setForeground(Qt.white)
                code_item.setBackground(Qt.darkRed)
            else:
                status_item.setText("OK")
                status_item.setBackground(Qt.transparent)
                status_item.setForeground(Qt.green)
                code_item.setForeground(Qt.white)
                code_item.setBackground(Qt.transparent)

    def reset_placeholders(self):
        for row in range(self.error_table.rowCount()):
            status_item = self.error_table.item(row, 2)
            code_item = self.error_table.item(row, 1)
            status_item.setText("-")
            status_item.setBackground(Qt.transparent)
            status_item.setForeground(Qt.gray)
            code_item.setBackground(Qt.transparent)
            code_item.setForeground(Qt.gray)
