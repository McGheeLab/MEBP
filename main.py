# ///////////////////////////////////////////////////////////////
#
# BY: WANDERSON M.PIMENTA
# PROJECT MADE WITH: Qt Designer and PySide6
# V: 1.0.0
#
# This project can be used freely for all uses, as long as they maintain the
# respective credits only in the Python scripts, any information in the visual
# interface (GUI) can be modified without any implication.
#
# There are limitations on Qt licenses if you want to use your products
# commercially, I recommend reading them on the official website:
# https://doc.qt.io/qtforpython/licenses.html
#
# ///////////////////////////////////////////////////////////////

# IMPORT PACKAGES AND MODULES
# ///////////////////////////////////////////////////////////////
from gui.uis.windows.main_window.functions_main_window import *
import sys
import os
import ctypes

# IMPORT APP CONTROLLER (manages all stage devices)
# ///////////////////////////////////////////////////////////////
from SupportClasses.ProcessCommand import AppController

# IMPORT all of the settings, windows, and widgets
# ///////////////////////////////////////////////////////////////
from qt_core import *
from gui.core.json_settings import Settings
from gui.uis.windows.main_window import *
from gui.widgets import *


# AUTO ADJUST DPI
# ///////////////////////////////////////////////////////////////
# Adjust QT DPI based on Windows display settings in a more dynamic way
if sys.platform.startswith("win"):
    try:
        user32 = ctypes.windll.user32
        # Mark the process as DPI aware for correct scaling
        user32.SetProcessDPIAware()
        dpi = user32.GetDpiForSystem()  # Windows returns the system DPI (e.g., 96, 144, etc.)
        scale = dpi / 96  # 96 DPI is considered 100%
    except Exception:
        scale = 1.0
else:
    scale = 1.0

os.environ["QT_FONT_DPI"] = "96"
if scale > 1.0:
    os.environ["QT_SCALE_FACTOR"] = str(scale)

# MAIN WINDOW
# ///////////////////////////////////////////////////////////////
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # SETUP MAIN WINDOw
        # Load widgets from "gui\uis\main_window\ui_main.py"
        # ///////////////////////////////////////////////////////////////
        self.ui = UI_MainWindow()
        self.ui.setup_ui(self)

        # LOAD SETTINGS
        # ///////////////////////////////////////////////////////////////
        settings = Settings()
        self.settings = settings.items

        # Instantiate AppController early.
        self.app_controller = AppController()  # if you pass a flag; adjust as needed
        
        # Now, create an XboxPoller instance. Since MainWindow is a QObject in the main thread,
        # it is safe to create a QTimer here.
        self.xbox_poller = XboxPoller(self.app_controller.xbox_queue, self.app_controller.processor, parent=self)
        self.xbox_poller.start()
        
        # SETUP MAIN WINDOW
        # ///////////////////////////////////////////////////////////////
        self.hide_grips = True # Show/Hide resize grips
        SetupMainWindow.setup_gui(self)
        SetupMainWindow.page1(self)
        SetupMainWindow.page2(self)
        SetupMainWindow.page3(self)
        SetupMainWindow.left_column(self)
        SetupMainWindow.right_column(self)

        # SHOW MAIN WINDOW
        # ///////////////////////////////////////////////////////////////
        self.show()
    
    def closeEvent(self, event):
        self.app_controller.stop()
        self.xbox_poller.stop()
        event.accept()
        
    # LEFT MENU BTN IS CLICKED
    # Run function when btn is clicked
    # Check funtion by object name / btn_id
    # ///////////////////////////////////////////////////////////////
    def btn_clicked(self):
        # GET BT CLICKED
        btn = SetupMainWindow.setup_btns(self)

        # Remove Selection If Clicked By "btn_close_left_column"
        if btn.objectName() != "btn_settings":
            self.ui.left_menu.deselect_all_tab()

        # Get Title Bar Btn And Reset Active         
        top_settings = MainFunctions.get_title_bar_btn(self, "btn_top_settings")
        top_settings.set_active(False)

        # LEFT MENU
        # ///////////////////////////////////////////////////////////////
        
        # HOME BTN
        if btn.objectName() == "btn_home":
            # Select Menu
            self.ui.left_menu.select_only_one(btn.objectName())

            # Load Page 1
            MainFunctions.set_page(self, self.ui.load_pages.page_1)
            
        if btn.objectName() == "btn_home2":
            # Select Menu
            self.ui.left_menu.select_only_one(btn.objectName())

            # Load Page 1
            MainFunctions.set_page(self, self.ui.load_pages.page_2)
            
        if btn.objectName() == "btn_home3":
            # Select Menu
            self.ui.left_menu.select_only_one(btn.objectName())

            # Load Page 1
            MainFunctions.set_page(self, self.ui.load_pages.page_3)
        
        if btn.objectName() == "btn_settings" or btn.objectName() == "btn_close_left_column":
            #check if left column is visible
            if not MainFunctions.left_column_is_visible(self):
                
                # Show / Hide
                MainFunctions.toggle_left_column(self)
                self.ui.left_menu.select_only_one_tab(btn.objectName())
            else:
                if btn.objectName() == "btn_close_left_column":
                    self.ui.left_menu.deselect_all_tab()
                    # Show / Hide
                    MainFunctions.toggle_left_column(self)
                self.ui.left_menu.select_only_one_tab(btn.objectName())
            

        # WIDGETS BTN
        if btn.objectName() == "btn_widgets":
            # Select Menu
            self.ui.left_menu.select_only_one(btn.objectName())

            # Load Page 2
            MainFunctions.set_page(self, self.ui.load_pages.page_2)

        # LOAD USER PAGE
        if btn.objectName() == "btn_add_user":
            # Select Menu
            self.ui.left_menu.select_only_one(btn.objectName())

            # Load Page 3 
            MainFunctions.set_page(self, self.ui.load_pages.page_3)

        # BOTTOM INFORMATION
        if btn.objectName() == "btn_info":
            # CHECK IF LEFT COLUMN IS VISIBLE
            if not MainFunctions.left_column_is_visible(self):
                self.ui.left_menu.select_only_one_tab(btn.objectName())

                # Show / Hide
                MainFunctions.toggle_left_column(self)
                self.ui.left_menu.select_only_one_tab(btn.objectName())
            else:
                if btn.objectName() == "btn_close_left_column":
                    self.ui.left_menu.deselect_all_tab()
                    # Show / Hide
                    MainFunctions.toggle_left_column(self)
                
                self.ui.left_menu.select_only_one_tab(btn.objectName())

            # Change Left Column Menu
            if btn.objectName() != "btn_close_left_column":
                MainFunctions.set_left_column_menu(
                    self, 
                    menu = self.ui.left_column.menus.menu_2,
                    title = "Info tab",
                    icon_path = Functions.set_svg_icon("icon_info.svg")
                )
        # SETTINGS LEFT
        if btn.objectName() == "btn_settings" or btn.objectName() == "btn_close_left_column":
            # CHECK IF LEFT COLUMN IS VISIBLE
            if not MainFunctions.left_column_is_visible(self):
                # Show / Hide
                MainFunctions.toggle_left_column(self)
                self.ui.left_menu.select_only_one_tab(btn.objectName())
            else:
                if btn.objectName() == "btn_close_left_column":
                    self.ui.left_menu.deselect_all_tab()
                    # Show / Hide
                    MainFunctions.toggle_left_column(self)
                self.ui.left_menu.select_only_one_tab(btn.objectName())

            # Change Left Column Menu
            if btn.objectName() != "btn_close_left_column":
                MainFunctions.set_left_column_menu(
                    self, 
                    menu = self.ui.left_column.menus.menu_1,
                    title = "Settings Left Column",
                    icon_path = Functions.set_svg_icon("icon_settings.svg")
                )
        
        # TITLE BAR MENU
        # ///////////////////////////////////////////////////////////////
        
        # SETTINGS TITLE BAR
        if btn.objectName() == "btn_top_settings":
            # Toogle Active
            if not MainFunctions.right_column_is_visible(self):
                btn.set_active(True)

                # Show / Hide
                MainFunctions.toggle_right_column(self)
            else:
                btn.set_active(False)

                # Show / Hide
                MainFunctions.toggle_right_column(self)

            # Get Left Menu Btn            
            top_settings = MainFunctions.get_left_menu_btn(self, "btn_settings")
            top_settings.set_active_tab(False)            

        # DEBUG
        print(f"Button {btn.objectName()}, clicked!")

    # LEFT MENU BTN IS RELEASED
    # Run function when btn is released
    # Check funtion by object name / btn_id
    # ///////////////////////////////////////////////////////////////
    def btn_released(self):
        # GET BT CLICKED
        btn = SetupMainWindow.setup_btns(self)

        # DEBUG
        print(f"Button {btn.objectName()}, released!")

    # RESIZE EVENT
    # ///////////////////////////////////////////////////////////////
    def resizeEvent(self, event):
        SetupMainWindow.resize_grips(self)

    # MOUSE CLICK EVENTS
    # ///////////////////////////////////////////////////////////////
    def mousePressEvent(self, event):
        # SET DRAG POS WINDOW
        self.dragPos = event.globalPos()


class XboxPoller(QObject):
    def __init__(self, queue, processor, parent=None):
        super().__init__(parent)
        self.queue = queue
        self.processor = processor
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
    
    def start(self):
        # Start polling every 50ms.
        self.timer.start(50)
    
    def stop(self):
        self.timer.stop()
    
    def poll(self):
        while not self.queue.empty():
            msg = self.queue.get()
            # Process debug messages.
            if "debug" in msg:
                self.processor.add_command("debug", message=msg["debug"])
            # Process button messages.
            elif "button" in msg:
                self.processor.add_command(msg["command"], button=msg["button"])
            # Process axis messages.
            elif "axis" in msg:
                self.processor.add_command(msg["command"], axis=msg["axis"], average=msg["average"])
            # Process DPad messages.
            elif "dpad" in msg:
                self.processor.add_command(msg["command"], direction=msg["dpad"])

# SETTINGS WHEN TO START
# Set the initial class and also additional parameters of the "QApplication" class
# ///////////////////////////////////////////////////////////////
if __name__ == "__main__":
    # APPLICATION
    # ///////////////////////////////////////////////////////////////
    app = QApplication(sys.argv)
    app.setWindowIcon(QIcon("icon.ico"))
    window = MainWindow()

    # EXEC APP
    # ///////////////////////////////////////////////////////////////
    sys.exit(app.exec_())