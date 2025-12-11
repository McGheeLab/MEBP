# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'main_pageszAanYl.ui'
##
## Created by: Qt User Interface Compiler version 6.4.3
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from qt_core import *

class Ui_MainPages(object):
    def setupUi(self, MainPages):
        if not MainPages.objectName():
            MainPages.setObjectName(u"MainPages")
        MainPages.resize(860, 600)
        self.main_pages_layout = QVBoxLayout(MainPages)
        self.main_pages_layout.setSpacing(0)
        self.main_pages_layout.setObjectName(u"main_pages_layout")
        self.main_pages_layout.setContentsMargins(5, 5, 5, 5)
        
        # Make the window frameless
        self.pages = QStackedWidget(MainPages)
        self.pages.setObjectName(u"pages")
        
        # Page 1
        self.page_1 = QWidget()
        self.page_1.setObjectName(u"Overview")
        self.page_1.setStyleSheet(u"font-size: 14pt;")
        self.page_1_layout = QVBoxLayout(self.page_1)
        self.page_1_layout.setSpacing(5)
        self.page_1_layout.setObjectName(u"page_1_layout")
        self.page_1_layout.setContentsMargins(5, 5, 5, 5)
        
        # Page 2
        self.page_2 = QWidget()
        self.page_2.setObjectName(u"Calibration")
        self.page_2.setStyleSheet(u"font-size: 14pt;")
        self.page_2_layout = QVBoxLayout(self.page_2)
        self.page_2_layout.setSpacing(5)
        self.page_2_layout.setObjectName(u"page_2_layout")
        self.page_2_layout.setContentsMargins(5, 5, 5, 5)
        
        # Page 3
        self.page_3 = QWidget()
        self.page_3.setObjectName(u"Print Setup")
        self.page_3.setStyleSheet(u"font-size: 14pt;")
        self.page_3_layout = QVBoxLayout(self.page_3)
        self.page_3_layout.setObjectName(u"page_3_layout")
        
        # Page 4
        self.page_4 = QWidget()
        self.page_4.setObjectName(u"Print Setup")
        self.page_4.setStyleSheet(u"font-size: 14pt;")
        self.page_4_layout = QVBoxLayout(self.page_4)
        self.page_4_layout.setObjectName(u"page_4_layout")
        
        # Page 5
        self.page_5 = QWidget()
        self.page_5.setObjectName(u"Print Setup")
        self.page_5.setStyleSheet(u"font-size: 14pt;")
        self.page_5_layout = QVBoxLayout(self.page_5)
        self.page_5_layout.setObjectName(u"page_4_layout")
        

        # Page 6 - Controller Layout
        self.page_6 = QWidget()
        self.page_6.setObjectName(u"ControllerLayout")
        self.page_6.setStyleSheet(u"font-size: 14pt;")
        self.page_6_layout = QVBoxLayout(self.page_6)
        self.page_6_layout.setObjectName(u"page_4_layout")

        
        # Add pages to the stack
        self.pages.addWidget(self.page_1)
        self.pages.addWidget(self.page_2)
        self.pages.addWidget(self.page_3)
        self.pages.addWidget(self.page_4)
        self.pages.addWidget(self.page_5)
        self.pages.addWidget(self.page_6)
        
        self.main_pages_layout.addWidget(self.pages)


        self.retranslateUi(MainPages)

        self.pages.setCurrentIndex(2)


        QMetaObject.connectSlotsByName(MainPages)
    # setupUi

    def retranslateUi(self, MainPages):
        MainPages.setWindowTitle(QCoreApplication.translate("MainPages", u"Form", None))
    # retranslateUi

