import os
import sys
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QFileDialog, QSlider, QScrollArea, QCheckBox
)
from PyQt5.QtCore import Qt, QRect, QPoint
from PyQt5.QtGui import QPainter, QColor, QImage, QPen, QIntValidator, QFont


class GridWidget(QWidget):
    def __init__(self, rows, cols, cell_size=15):
        super().__init__()
        self.rows = rows
        self.cols = cols
        self.cell_size = cell_size
        self.grid = [[1 for _ in range(cols)] for _ in range(rows)]
        self.drawing = False
        self.paint_mode = True
        self.brush_size = 3
        self.brush_shape_square = False
        self.cursor_pos = None  # for preview
        self.setMouseTracking(True)

        self.update_widget_size()

    def update_widget_size(self):
        self.setFixedSize(self.cols * self.cell_size, self.rows * self.cell_size)

    def paintEvent(self, event):
        painter = QPainter(self)
        rect = event.rect()
        painter.fillRect(rect, Qt.white)

        # Draw cells
        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r][c] == 0:
                    painter.fillRect(
                        c * self.cell_size,
                        r * self.cell_size,
                        self.cell_size,
                        self.cell_size,
                        Qt.black
                    )

        # Grid lines
        painter.setPen(QColor(200, 200, 200))
        for r in range(self.rows + 1):
            painter.drawLine(0, r * self.cell_size, self.cols * self.cell_size, r * self.cell_size)
        for c in range(self.cols + 1):
            painter.drawLine(c * self.cell_size, 0, c * self.cell_size, self.rows * self.cell_size)

        # Center cross
        pen = QPen(Qt.gray, 1)
        painter.setPen(pen)
        mid_x = int(self.cols * self.cell_size / 2)
        mid_y = int(self.rows * self.cell_size / 2)
        painter.drawLine(mid_x, 0, mid_x, self.rows * self.cell_size)
        painter.drawLine(0, mid_y, self.cols * self.cell_size, mid_y)

        # Brush preview
        if self.cursor_pos:
            self.draw_brush_preview(painter, self.cursor_pos)

    def draw_brush_preview(self, painter, pos):
        col = pos.x() // self.cell_size
        row = pos.y() // self.cell_size
        if not (0 <= row < self.rows and 0 <= col < self.cols):
            return

        painter.setPen(Qt.NoPen)
        preview_color = QColor(128, 128, 128, 100)  # semi-transparent gray
        painter.setBrush(preview_color)

        brush_range = self.get_brush_range(row, col)

        for r, c in brush_range:
            if 0 <= r < self.rows and 0 <= c < self.cols:
                painter.drawRect(
                    c * self.cell_size,
                    r * self.cell_size,
                    self.cell_size,
                    self.cell_size
                )

    def get_brush_range(self, center_row, center_col):
        """Calculate brush range for given size"""
        brush_range = []
        
        if self.brush_size == 1:
            brush_range = [(center_row, center_col)]
        elif self.brush_size == 2:
            # 2x2 square
            for r in range(center_row - 1, center_row + 1):
                for c in range(center_col - 1, center_col + 1):
                    brush_range.append((r, c))
        elif self.brush_size == 3:
            # 3x3 square
            for r in range(center_row - 1, center_row + 2):
                for c in range(center_col - 1, center_col + 2):
                    brush_range.append((r, c))
        elif self.brush_size == 4:
            # 4x4 square
            for r in range(center_row - 2, center_row + 2):
                for c in range(center_col - 2, center_col + 2):
                    brush_range.append((r, c))
        elif self.brush_size == 5:
            # 5x5 square
            for r in range(center_row - 2, center_row + 3):
                for c in range(center_col - 2, center_col + 3):
                    brush_range.append((r, c))
        elif self.brush_size == 6:
            # 6x6 square
            for r in range(center_row - 3, center_row + 3):
                for c in range(center_col - 3, center_col + 3):
                    brush_range.append((r, c))
        elif self.brush_size == 7:
            # 7x7 square
            for r in range(center_row - 3, center_row + 4):
                for c in range(center_col - 3, center_col + 4):
                    brush_range.append((r, c))
        elif self.brush_size == 8:
            # 8x8 square
            for r in range(center_row - 4, center_row + 4):
                for c in range(center_col - 4, center_col + 4):
                    brush_range.append((r, c))
        elif self.brush_size == 9:
            # 9x9 square
            for r in range(center_row - 4, center_row + 5):
                for c in range(center_col - 4, center_col + 5):
                    brush_range.append((r, c))
        elif self.brush_size == 10:
            # 10x10 square
            for r in range(center_row - 5, center_row + 5):
                for c in range(center_col - 5, center_col + 5):
                    brush_range.append((r, c))
        
        # Apply circle shape if selected
        if not self.brush_shape_square:
            filtered_range = []
            radius = self.brush_size // 2
            for r, c in brush_range:
                # Calculate distance from center
                dr = r - center_row
                dc = c - center_col
                if dr*dr + dc*dc <= radius*radius:
                    filtered_range.append((r, c))
            brush_range = filtered_range
            
        return brush_range

    def apply_brush(self, pos):
        col = pos.x() // self.cell_size
        row = pos.y() // self.cell_size
        if not (0 <= row < self.rows and 0 <= col < self.cols):
            return

        brush_range = self.get_brush_range(row, col)
        
        for r, c in brush_range:
            if 0 <= r < self.rows and 0 <= c < self.cols:
                new_val = 0 if self.paint_mode else 1
                if self.grid[r][c] != new_val:
                    self.grid[r][c] = new_val
        
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.paint_mode = True
            self.drawing = True
            self.apply_brush(event.pos())
        elif event.button() == Qt.RightButton:
            self.paint_mode = False
            self.drawing = True
            self.apply_brush(event.pos())

    def mouseMoveEvent(self, event):
        self.cursor_pos = event.pos()
        if self.drawing:
            self.apply_brush(event.pos())
        self.update()

    def mouseReleaseEvent(self, event):
        self.drawing = False

    def leaveEvent(self, event):
        self.cursor_pos = None
        self.update()

    def clear_grid(self):
        self.grid = [[1 for _ in range(self.cols)] for _ in range(self.rows)]
        self.update()

    def resize_grid(self, new_rows, new_cols):
        # Create new grid with new dimensions
        new_grid = [[1 for _ in range(new_cols)] for _ in range(new_rows)]
        
        # Calculate center offsets for centering content
        old_center_row = self.rows // 2
        old_center_col = self.cols // 2
        new_center_row = new_rows // 2
        new_center_col = new_cols // 2
        
        # Copy existing content centered
        for r in range(self.rows):
            for c in range(self.cols):
                # Calculate new position centered
                new_r = r - old_center_row + new_center_row
                new_c = c - old_center_col + new_center_col
                
                # Only copy if within new grid bounds
                if 0 <= new_r < new_rows and 0 <= new_c < new_cols:
                    new_grid[new_r][new_c] = self.grid[r][c]
        
        self.rows = new_rows
        self.cols = new_cols
        self.grid = new_grid
        self.update_widget_size()
        self.update()

    def export_binary_image(self, filename):
        img = QImage(self.cols, self.rows, QImage.Format_Mono)
        img.fill(1)
        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r][c] == 0:
                    img.setPixel(c, r, 0)
        img.save(filename)

    def load_binary_image(self, filename):
        img = QImage(filename)
        if img.isNull():
            print("Failed to load image")
            return
        img = img.convertToFormat(QImage.Format_Mono)
        self.rows = img.height()
        self.cols = img.width()
        self.grid = [[1 for _ in range(self.cols)] for _ in range(self.rows)]
        for r in range(self.rows):
            for c in range(self.cols):
                if img.pixelIndex(c, r) == 0:
                    self.grid[r][c] = 0
        self.update_widget_size()
        self.update()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Binary Shape Painter")

        # possible alternative to setValidator
        #self.rows_edit.setMaxLength(4)  # max digits
        int_validator = QIntValidator(1, 1920, self)

        #row_label = QLabel("Rows:")
        #row_label.setFixedHeight(40)
        #font = QFont() # change size via font
        #font.setPointSize(18)
        #row_label.setFont(font)

        self.rows_label = QLabel("Rows")
        self.rows_label.setFixedWidth(50)
        self.rows_label.setAlignment(Qt.AlignCenter)
        
        self.rows_edit = QLineEdit("32")
        self.rows_edit.setValidator(int_validator)
        self.rows_edit.setFixedWidth(50)
        self.rows_edit.setAlignment(Qt.AlignCenter)

        self.cols_label = QLabel("Columns")
        self.cols_label.setFixedWidth(50)
        self.cols_label.setAlignment(Qt.AlignCenter)
        
        self.cols_edit = QLineEdit("32")
        self.cols_edit.setValidator(int_validator)
        self.cols_edit.setFixedWidth(50)
        self.cols_edit.setAlignment(Qt.AlignCenter)

        self.lock_label = QLabel("Lock")
        self.lock_label.setFixedWidth(50)
        self.lock_label.setAlignment(Qt.AlignCenter)

        self.lock_button = QCheckBox()
        self.lock_button.setFixedWidth(35)
        self.lock_button.setChecked(True)

        self.brush_slider_label = QLabel("Brush Size: 3")
        self.brush_slider_label.setFixedWidth(80)

        self.brush_slider = QSlider(Qt.Horizontal)
        self.brush_slider.setRange(1, 10)
        self.brush_slider.setValue(3)
        self.brush_slider.setFixedWidth(150)

        self.brush_shape_label = QLabel("Toggle Brush:")
        self.brush_shape_label.setFixedWidth(80)
        self.brush_shape_btn = QPushButton("Circle")
        self.brush_shape_btn.setCheckable(True)

        self.generate_btn = QPushButton("Generate Grid")
        self.clear_btn = QPushButton("Clear Grid")

        self.export_btn = QPushButton("Export")
        self.export_btn.setFixedWidth(200)
        self.load_btn = QPushButton("Load")
        self.load_btn.setFixedWidth(200)

        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)

        self.grid_widget = GridWidget(32, 32)
        self.scroll_area.setWidget(self.grid_widget)

        self.init_ui()
        self.bind_events()
        self.resize_window()

    def init_ui(self):
        main_widget = QWidget()
        layout = QVBoxLayout(main_widget)
        #row1.addStretch()

        row1 = QHBoxLayout()
        row1.addWidget(self.rows_label)
        row1.addWidget(self.cols_label)
        row1.addWidget(self.lock_label)
        row1.addWidget(self.clear_btn)

        row2 = QHBoxLayout()
        row2.addWidget(self.cols_edit)
        row2.addWidget(self.rows_edit)
        row2.addSpacing(15)
        row2.addWidget(self.lock_button)
        row2.addWidget(self.generate_btn)
        
        row3 = QHBoxLayout()
        row3.addWidget(self.brush_slider_label)
        row3.addWidget(self.brush_slider)
        row3.addWidget(self.brush_shape_label)
        row3.addWidget(self.brush_shape_btn)

        row_bottom = QHBoxLayout()
        row_bottom.addWidget(self.load_btn)
        row_bottom.addStretch()
        row_bottom.addWidget(self.export_btn)

        layout.addLayout(row1)
        layout.addLayout(row2)
        layout.addLayout(row3)
        layout.addWidget(self.scroll_area)
        layout.addLayout(row_bottom)

        self.setCentralWidget(main_widget)

    def bind_events(self):
        self.brush_shape_btn.toggled.connect(self.toggle_brush_shape)
        self.brush_slider.valueChanged.connect(self.update_brush_size)
        self.clear_btn.clicked.connect(self.grid_widget.clear_grid)
        self.generate_btn.clicked.connect(self.generate_new_grid)
        self.export_btn.clicked.connect(self.export_image)
        self.load_btn.clicked.connect(self.load_image)
        self.rows_edit.textChanged.connect(self.sync_rows_cols)
        self.cols_edit.textChanged.connect(self.sync_cols_rows)

    def toggle_brush_shape(self, checked):
        self.grid_widget.brush_shape_square = checked
        if checked:
            self.brush_shape_btn.setText("Square")
        else:
            self.brush_shape_btn.setText("Circle")
        self.grid_widget.update()

    def update_brush_size(self, value):
        self.grid_widget.brush_size = value
        self.brush_slider_label.setText(f"Brush Size: {value}")
        self.grid_widget.update()

    def generate_new_grid(self):
        try:
            new_rows = int(self.rows_edit.text())
            new_cols = int(self.cols_edit.text())
            if new_rows > 0 and new_cols > 0:
                self.grid_widget.resize_grid(new_rows, new_cols)
                self.resize_window()
        except ValueError:
            print("Please enter valid numbers for rows and columns")

    def sync_rows_cols(self):
        if self.lock_button.isChecked():
            self.cols_edit.setText(self.rows_edit.text())

    def sync_cols_rows(self):
        if self.lock_button.isChecked():
            self.rows_edit.setText(self.cols_edit.text())

    def resize_window(self):
        max_cells = 64
        cell_size = self.grid_widget.cell_size
        
        # Always set the grid widget to its actual size
        self.grid_widget.setFixedSize(self.grid_widget.cols * cell_size, self.grid_widget.rows * cell_size)
        
        # Configure scroll area behavior based on grid size
        if self.grid_widget.rows <= max_cells and self.grid_widget.cols <= max_cells:
            # No scrolling needed - window can grow to fit
            self.scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            self.scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            window_width = self.grid_widget.cols * cell_size + 20
            window_height = self.grid_widget.rows * cell_size + 145
        else:
            # Scrolling needed - cap window size and enable scroll bars
            self.scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
            self.scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
            # Set a fixed maximum window size
            window_width = max_cells * cell_size + 20
            window_height = max_cells * cell_size + 145
        
        self.resize(window_width, window_height)

    def export_image(self):
        io_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../io"))
        filename, _ = QFileDialog.getSaveFileName(
            parent=self,
            caption="Save Binary Image",
            directory=os.path.join(io_path, ".png"),
            filter="PNG Image (*.png);;All Files (*)"
        )

        if filename:
            self.grid_widget.export_binary_image(filename)

    def load_image(self):
        io_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../io"))
        filename, _ = QFileDialog.getOpenFileName(self, "Open Binary Image", io_path, "PNG Files (*.png);;All Files (*)")
        if filename:
            self.grid_widget.load_binary_image(filename)
            self.resize_window()


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()