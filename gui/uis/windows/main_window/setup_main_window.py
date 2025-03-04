# ///////////////////////////////////////////////////////////////
# IMPORTS
# ///////////////////////////////////////////////////////////////

# Import custom widgets and modules
from gui.widgets.py_table_widget.py_table_widget import PyTableWidget
from .functions_main_window import *  # Functions for main window actions
import sys
import os

# Import Qt core components
from qt_core import *

# Import settings and theme information
from gui.core.json_settings import Settings
from gui.core.json_themes import Themes

# Import all custom widgets used by the application
from gui.widgets import *

# Load the UI for the main window (created using Qt Designer)
from .ui_main import *


# ///////////////////////////////////////////////////////////////
# CLASS: SetupMainWindow
# Description:
#   This class sets up the main window's configuration,
#   menus, and widget placement.
# ///////////////////////////////////////////////////////////////
class SetupMainWindow:
    def __init__(self):
        # Initialize the UI from the UI_MainWindow class
        self.ui = UI_MainWindow()
        self.ui.setup_ui(self)

    # ///////////////////////////////////////////////////////////////
    # LEFT MENUS CONFIGURATION
    # ///////////////////////////////////////////////////////////////
    add_left_menus = [
        {
            "btn_icon": "icon_home.svg",
            "btn_id": "btn_home",
            "btn_text": "Home",
            "btn_tooltip": "Home page",
            "show_top": True,
            "is_active": True,
        },
        {
            "btn_icon": "icon_online.svg",
            "btn_id": "btn_home2",
            "btn_text": "pg2",
            "btn_tooltip": "Open page 2",
            "show_top": True,
            "is_active": False,
        },
        {
            "btn_icon": "icon_signal.svg",
            "btn_id": "btn_home3",
            "btn_text": "pg3",
            "btn_tooltip": "Open page 3",
            "show_top": True,
            "is_active": False,
        },
        {
            "btn_icon": "icon_widgets.svg",
            "btn_id": "btn_home4",
            "btn_text": "pg4",
            "btn_tooltip": "Open page 4",
            "show_top": True,
            "is_active": False,
        },
        {
            "btn_icon": "icon_settings.svg",
            "btn_id": "btn_settings",
            "btn_text": "Settings",
            "btn_tooltip": "Open settings",
            "show_top": False,
            "is_active": False,
        }
    ]

    # ///////////////////////////////////////////////////////////////
    # TITLE BAR MENUS CONFIGURATION
    # ///////////////////////////////////////////////////////////////
    add_title_bar_menus = [
        {
            "btn_icon": "icon_settings.svg",
            "btn_id": "btn_top_settings",
            "btn_tooltip": "Top settings",
            "is_active": False,
        },
    ]

    # ///////////////////////////////////////////////////////////////
    # METHOD: setup_btns
    # Description:
    #   Returns the widget/button in the title bar,
    #   left menu, or left column that has sent a signal.
    # ///////////////////////////////////////////////////////////////
    def setup_btns(self):
        if self.ui.title_bar.sender() != None:
            return self.ui.title_bar.sender()
        elif self.ui.left_menu.sender() != None:
            return self.ui.left_menu.sender()
        elif self.ui.left_column.sender() != None:
            return self.ui.left_column.sender()

    # RESIZE GRIPS AND CHANGE POSITION
    # Resize or change position when window is resized
    # ///////////////////////////////////////////////////////////////
    def resize_grips(self):
        if self.settings["custom_title_bar"]:
            self.left_grip.setGeometry(5, 10, 10, self.height())
            self.right_grip.setGeometry(self.width() - 15, 10, 10, self.height())
            self.top_grip.setGeometry(5, 5, self.width() - 10, 10)
            self.bottom_grip.setGeometry(5, self.height() - 15, self.width() - 10, 10)
            self.top_right_grip.setGeometry(self.width() - 20, 5, 15, 15)
            self.bottom_left_grip.setGeometry(5, self.height() - 20, 15, 15)
            self.bottom_right_grip.setGeometry(self.width() - 20, self.height() - 20, 15, 15)
    
    # SETUP GUI
    # ///////////////////////////////////////////////////////////////
    def setup_gui(self):
        
            # APP TITLE
        # ///////////////////////////////////////////////////////////////
        self.setWindowTitle(self.settings["app_name"])
        
        # REMOVE TITLE BAR
        # ///////////////////////////////////////////////////////////////
        if self.settings["custom_title_bar"]:
            self.setWindowFlag(Qt.FramelessWindowHint)
            self.setAttribute(Qt.WA_TranslucentBackground)

        # ADD GRIPS
        # ///////////////////////////////////////////////////////////////
        if self.settings["custom_title_bar"]:
            self.left_grip = PyGrips(self, "left", self.hide_grips)
            self.right_grip = PyGrips(self, "right", self.hide_grips)
            self.top_grip = PyGrips(self, "top", self.hide_grips)
            self.bottom_grip = PyGrips(self, "bottom", self.hide_grips)
            self.top_left_grip = PyGrips(self, "top_left", self.hide_grips)
            self.top_right_grip = PyGrips(self, "top_right", self.hide_grips)
            self.bottom_left_grip = PyGrips(self, "bottom_left", self.hide_grips)
            self.bottom_right_grip = PyGrips(self, "bottom_right", self.hide_grips)

        # LEFT MENUS / GET SIGNALS WHEN LEFT MENU BTN IS CLICKED / RELEASED
        # ///////////////////////////////////////////////////////////////
        # ADD MENUS
        self.ui.left_menu.add_menus(SetupMainWindow.add_left_menus)

        # SET SIGNALS
        self.ui.left_menu.clicked.connect(self.btn_clicked)
        self.ui.left_menu.released.connect(self.btn_released)

        # TITLE BAR / ADD EXTRA BUTTONS
        # ///////////////////////////////////////////////////////////////
        # ADD MENUS
        self.ui.title_bar.add_menus(SetupMainWindow.add_title_bar_menus)

        # SET SIGNALS
        self.ui.title_bar.clicked.connect(self.btn_clicked)
        self.ui.title_bar.released.connect(self.btn_released)

        # ADD Title
        self.ui.title_bar.set_title("Microscope Enabled 3D Bioprinter")

        #   LEFT COLUMN SET SIGNALS
        # ///////////////////////////////////////////////////////////////
        self.ui.left_column.clicked.connect(self.btn_clicked)
        self.ui.left_column.released.connect(self.btn_released)

        # SET INITIAL PAGE / SET LEFT AND RIGHT COLUMN MENUS
        # ///////////////////////////////////////////////////////////////
        MainFunctions.set_page(self, self.ui.load_pages.page_1)
        MainFunctions.set_left_column_menu(
            self,
            menu = self.ui.left_column.menus.menu_1,
            title = "Settings Left Column",
            icon_path = Functions.set_svg_icon("icon_settings.svg")
        )
        MainFunctions.set_right_column_menu(self, self.ui.right_column.menu_1)
        
        # LOAD SETTINGS
        # ///////////////////////////////////////////////////////////////
        settings = Settings()
        self.settings = settings.items

        # LOAD THEME COLOR
        # ///////////////////////////////////////////////////////////////
        themes = Themes()
        self.themes = themes.items
    
    # SETUP EACH PAGE 
    # ///////////////////////////////////////////////////////////////
    def left_column(self):
        # Make objects
        self.btn = PyPushButton(
                text="Button 1",
                radius=8,
                color=self.themes["app_color"]["text_foreground"],
                bg_color = self.themes["app_color"]["dark_one"],
                bg_color_hover= self.themes["app_color"]["dark_four"],
                bg_color_pressed= self.themes["app_color"]["dark_three"],
            )
            
        self.btn.setMinimumHeight(40)
        self.btn.setMaximumHeight(40)
        
        #add objects to layout
        self.ui.left_column.menus.btn1_layout.addWidget(self.btn)
    
    def right_column(self):
        # Make objects
        
        # Add objects to layout
        
        # Set signals
        
        # Connect signals
        
        pass
        
    def page1(self):
        # Now, get the StageMonitorWidget from your AppController
        # (Assuming AppController creates the StageMonitorWidget and stores it in an attribute)
        stage_monitor_widget = StageMonitorWidget(self.app_controller)
        
        # Add the StageMonitorWidget to page1's layout.
        self.ui.load_pages.page_1_layout.addWidget(stage_monitor_widget)

    def page2(self):
        # Create an instance of the camera widget
        camera_widget = CameraWidget()
        # Add the camera widget to page2's layout
        self.ui.load_pages.page_2_layout.addWidget(camera_widget)       

    def page3(self):
        # Now, get the StageMonitorWidget from your AppController
        # (Assuming AppController creates the StageMonitorWidget and stores it in an attribute)
        print_settings_widget = PrintSettingsWidget(self.print_mgr)
        
        # Add the StageMonitorWidget to page1's layout.
        self.ui.load_pages.page_3_layout.addWidget(print_settings_widget)
    
    def page4(self):
        # Now, get the StageMonitorWidget from your AppController
        # (Assuming AppController creates the StageMonitorWidget and stores it in an attribute)
        print_monitor_widget = PrintMonitorWidget(self.print_mgr)
        
        # Add the StageMonitorWidget to page1's layout.
        self.ui.load_pages.page_4_layout.addWidget(print_monitor_widget)
              
