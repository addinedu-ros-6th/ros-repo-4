# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'interfacedMnnut.ui'
##
## Created by: Qt User Interface Compiler version 5.15.3
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *

from Custom_Widgets.Widgets import QCustomSlideMenu
from Custom_Widgets.Widgets import QCustomStackedWidget

import resources_rc

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1117, 732)
        MainWindow.setStyleSheet(u"*{\n"
"	border: none;\n"
"	background-color: transparent;\n"
"	background: transparent;\n"
"	padding: 0;\n"
"	margin: 0;\n"
"	color: #06558D;\n"
"}\n"
"#centralwidget{\n"
"	background-color: #F4F4F6;\n"
"}\n"
"#leftMenuSubContainer{\n"
"	\n"
"  background-color: #DEE4FA;\n"
"}\n"
"#leftMenuSubContainer QPushButton{\n"
"	text-align: left;\n"
"	padding: 5px 10px;\n"
"	border-top-left-radius: 10px;\n"
"	border-bottom-left-radius: 10px;\n"
"  margin-top: 5px;\n"
"}\n"
"#centerMenuSubContainer, #rightMenuSubContainer{\n"
"	background-color: #E6EAF8;\n"
"}\n"
"#frame_4, #frame_8, #popupNotificationSubContainer{\n"
"	background-color: #FFFFFF;\n"
"	border-radius: 10px;\n"
"}\n"
"#headerContainer, #footerContainer{\n"
"	background-color: #E6EAF8;\n"
"}\n"
"#card_1, #card_2, #card_3, #card_4, #controlBtnLayout, #zoomInOutBtnFrame, #card_5, #card_6, #frame_17, #frame_19\n"
"{\n"
"	background-color: #C7CAD9;\n"
"	border-radius: 20px;\n"
"}\n"
"\n"
"#label_status_b1, #label_status_b2, #label_status_b3, #label_status_b4, #"
                        "label_motor_b2, #label_motor_b3, #label_motor_b4, #label_lidar_b1, #label_lidar_b2, #label_lidar_b3, #label_lidar_b4\n"
"{\n"
"	border-radius: 10px;\n"
"	background-color: gray;\n"
"}\n"
"\n"
"#frame_mt_b1, #frame_li_b1, #frame_mt_b2, #frame_li_b2, #frame_mt_b3, #frame_li_b3, #frame_mt_b4, #frame_li_b4, #frame_mode_state_b1, #frame_mode_state_b2, #frame_mode_state_b3, #frame_mode_state_b4\n"
"{\n"
"	background-color: #FFFFFF;\n"
"	border-radius: 10px;\n"
"}\n"
"\n"
"#label_18, #label_10\n"
"{\n"
"	color: rgb(255,0,0);\n"
"}")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.horizontalLayout = QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setSpacing(0)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.leftMenuContainer = QCustomSlideMenu(self.centralwidget)
        self.leftMenuContainer.setObjectName(u"leftMenuContainer")
        self.leftMenuContainer.setMaximumSize(QSize(45, 16777215))
        self.verticalLayout = QVBoxLayout(self.leftMenuContainer)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.leftMenuSubContainer = QWidget(self.leftMenuContainer)
        self.leftMenuSubContainer.setObjectName(u"leftMenuSubContainer")
        self.verticalLayout_2 = QVBoxLayout(self.leftMenuSubContainer)
        self.verticalLayout_2.setSpacing(0)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.verticalLayout_2.setContentsMargins(5, 0, 0, 0)
        self.frame = QFrame(self.leftMenuSubContainer)
        self.frame.setObjectName(u"frame")
        self.frame.setFrameShape(QFrame.StyledPanel)
        self.frame.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_2 = QHBoxLayout(self.frame)
        self.horizontalLayout_2.setSpacing(0)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.menuBtn = QPushButton(self.frame)
        self.menuBtn.setObjectName(u"menuBtn")
        icon = QIcon()
        icon.addFile(u":/icons/icons/blueIcons/align-justify.svg", QSize(), QIcon.Normal, QIcon.Off)
        self.menuBtn.setIcon(icon)
        self.menuBtn.setIconSize(QSize(24, 24))
        self.menuBtn.setContentsMargins(-1, 10, -1, -1)

        self.horizontalLayout_2.addWidget(self.menuBtn)


        self.verticalLayout_2.addWidget(self.frame, 0, Qt.AlignTop)

        self.frame_2 = QFrame(self.leftMenuSubContainer)
        self.frame_2.setObjectName(u"frame_2")
        sizePolicy = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame_2.sizePolicy().hasHeightForWidth())
        self.frame_2.setSizePolicy(sizePolicy)
        self.frame_2.setFrameShape(QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QFrame.Raised)
        self.verticalLayout_3 = QVBoxLayout(self.frame_2)
        self.verticalLayout_3.setSpacing(0)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.verticalLayout_3.setContentsMargins(0, 10, 0, 10)
        self.homeBtn = QPushButton(self.frame_2)
        self.homeBtn.setObjectName(u"homeBtn")
        self.homeBtn.setStyleSheet(u"background-color: #F4F4F6;")
        icon1 = QIcon()
        icon1.addFile(u":/icons/icons/blueIcons/home.svg", QSize(), QIcon.Normal, QIcon.Off)
        self.homeBtn.setIcon(icon1)
        self.homeBtn.setIconSize(QSize(24, 24))

        self.verticalLayout_3.addWidget(self.homeBtn)

        self.monitorBtn = QPushButton(self.frame_2)
        self.monitorBtn.setObjectName(u"monitorBtn")
        icon2 = QIcon()
        icon2.addFile(u":/icons/icons/blueIcons/monitor.svg", QSize(), QIcon.Normal, QIcon.Off)
        self.monitorBtn.setIcon(icon2)
        self.monitorBtn.setIconSize(QSize(24, 24))

        self.verticalLayout_3.addWidget(self.monitorBtn)


        self.verticalLayout_2.addWidget(self.frame_2)

        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_2.addItem(self.verticalSpacer)

        self.frame_3 = QFrame(self.leftMenuSubContainer)
        self.frame_3.setObjectName(u"frame_3")
        self.frame_3.setFrameShape(QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QFrame.Raised)
        self.verticalLayout_4 = QVBoxLayout(self.frame_3)
        self.verticalLayout_4.setSpacing(0)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.verticalLayout_4.setContentsMargins(0, 10, 0, 10)
        self.settingsBtn = QPushButton(self.frame_3)
        self.settingsBtn.setObjectName(u"settingsBtn")
        icon3 = QIcon()
        icon3.addFile(u":/icons/icons/blueIcons/settings.svg", QSize(), QIcon.Normal, QIcon.Off)
        self.settingsBtn.setIcon(icon3)
        self.settingsBtn.setIconSize(QSize(24, 24))

        self.verticalLayout_4.addWidget(self.settingsBtn)

        self.infoBtn = QPushButton(self.frame_3)
        self.infoBtn.setObjectName(u"infoBtn")
        icon4 = QIcon()
        icon4.addFile(u":/icons/icons/blueIcons/info.svg", QSize(), QIcon.Normal, QIcon.Off)
        self.infoBtn.setIcon(icon4)
        self.infoBtn.setIconSize(QSize(24, 24))

        self.verticalLayout_4.addWidget(self.infoBtn)

        self.helpBtn = QPushButton(self.frame_3)
        self.helpBtn.setObjectName(u"helpBtn")
        icon5 = QIcon()
        icon5.addFile(u":/icons/icons/blueIcons/help-circle.svg", QSize(), QIcon.Normal, QIcon.Off)
        self.helpBtn.setIcon(icon5)
        self.helpBtn.setIconSize(QSize(24, 24))

        self.verticalLayout_4.addWidget(self.helpBtn)


        self.verticalLayout_2.addWidget(self.frame_3, 0, Qt.AlignBottom)


        self.verticalLayout.addWidget(self.leftMenuSubContainer)


        self.horizontalLayout.addWidget(self.leftMenuContainer, 0, Qt.AlignLeft)

        self.centerMenuContainer = QCustomSlideMenu(self.centralwidget)
        self.centerMenuContainer.setObjectName(u"centerMenuContainer")
        self.centerMenuContainer.setMinimumSize(QSize(200, 0))
        self.verticalLayout_5 = QVBoxLayout(self.centerMenuContainer)
        self.verticalLayout_5.setSpacing(0)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.verticalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.centerMenuSubContainer = QWidget(self.centerMenuContainer)
        self.centerMenuSubContainer.setObjectName(u"centerMenuSubContainer")
        self.centerMenuSubContainer.setMinimumSize(QSize(200, 0))
        self.verticalLayout_6 = QVBoxLayout(self.centerMenuSubContainer)
        self.verticalLayout_6.setSpacing(5)
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.verticalLayout_6.setContentsMargins(5, 5, 5, 5)
        self.frame_4 = QFrame(self.centerMenuSubContainer)
        self.frame_4.setObjectName(u"frame_4")
        self.frame_4.setFrameShape(QFrame.StyledPanel)
        self.frame_4.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_4 = QHBoxLayout(self.frame_4)
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.label_2 = QLabel(self.frame_4)
        self.label_2.setObjectName(u"label_2")
        sizePolicy1 = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.label_2.sizePolicy().hasHeightForWidth())
        self.label_2.setSizePolicy(sizePolicy1)
        self.label_2.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_4.addWidget(self.label_2)

        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_4.addItem(self.horizontalSpacer)

        self.closeCenterMenuBtn = QPushButton(self.frame_4)
        self.closeCenterMenuBtn.setObjectName(u"closeCenterMenuBtn")
        sizePolicy2 = QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.closeCenterMenuBtn.sizePolicy().hasHeightForWidth())
        self.closeCenterMenuBtn.setSizePolicy(sizePolicy2)
        self.closeCenterMenuBtn.setLayoutDirection(Qt.LeftToRight)
        icon6 = QIcon()
        icon6.addFile(u":/icons/icons/blueIcons/x-circle.svg", QSize(), QIcon.Normal, QIcon.Off)
        self.closeCenterMenuBtn.setIcon(icon6)
        self.closeCenterMenuBtn.setIconSize(QSize(24, 24))
        self.closeCenterMenuBtn.setAutoExclusive(False)

        self.horizontalLayout_4.addWidget(self.closeCenterMenuBtn)


        self.verticalLayout_6.addWidget(self.frame_4, 0, Qt.AlignTop)

        self.centerMenuPages = QCustomStackedWidget(self.centerMenuSubContainer)
        self.centerMenuPages.setObjectName(u"centerMenuPages")
        self.page = QWidget()
        self.page.setObjectName(u"page")
        self.verticalLayout_7 = QVBoxLayout(self.page)
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.label_3 = QLabel(self.page)
        self.label_3.setObjectName(u"label_3")
        font = QFont()
        font.setPointSize(13)
        self.label_3.setFont(font)
        self.label_3.setAlignment(Qt.AlignCenter)

        self.verticalLayout_7.addWidget(self.label_3)

        self.centerMenuPages.addWidget(self.page)
        self.page_2 = QWidget()
        self.page_2.setObjectName(u"page_2")
        self.verticalLayout_8 = QVBoxLayout(self.page_2)
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.label_4 = QLabel(self.page_2)
        self.label_4.setObjectName(u"label_4")
        font1 = QFont()
        font1.setPointSize(13)
        font1.setBold(True)
        font1.setUnderline(False)
        font1.setWeight(75)
        font1.setStrikeOut(False)
        self.label_4.setFont(font1)
        self.label_4.setAlignment(Qt.AlignCenter)

        self.verticalLayout_8.addWidget(self.label_4)

        self.textEdit = QTextEdit(self.page_2)
        self.textEdit.setObjectName(u"textEdit")

        self.verticalLayout_8.addWidget(self.textEdit)

        self.centerMenuPages.addWidget(self.page_2)
        self.page_3 = QWidget()
        self.page_3.setObjectName(u"page_3")
        self.verticalLayout_9 = QVBoxLayout(self.page_3)
        self.verticalLayout_9.setObjectName(u"verticalLayout_9")
        self.label_5 = QLabel(self.page_3)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setFont(font)
        self.label_5.setAlignment(Qt.AlignCenter)

        self.verticalLayout_9.addWidget(self.label_5)

        self.textEdit_2 = QTextEdit(self.page_3)
        self.textEdit_2.setObjectName(u"textEdit_2")

        self.verticalLayout_9.addWidget(self.textEdit_2)

        self.centerMenuPages.addWidget(self.page_3)

        self.verticalLayout_6.addWidget(self.centerMenuPages)


        self.verticalLayout_5.addWidget(self.centerMenuSubContainer)


        self.horizontalLayout.addWidget(self.centerMenuContainer)

        self.mainBodyContainer = QWidget(self.centralwidget)
        self.mainBodyContainer.setObjectName(u"mainBodyContainer")
        sizePolicy1.setHeightForWidth(self.mainBodyContainer.sizePolicy().hasHeightForWidth())
        self.mainBodyContainer.setSizePolicy(sizePolicy1)
        self.mainBodyContainer.setMinimumSize(QSize(710, 527))
        self.mainBodyContainer.setStyleSheet(u"")
        self.verticalLayout_10 = QVBoxLayout(self.mainBodyContainer)
        self.verticalLayout_10.setSpacing(0)
        self.verticalLayout_10.setObjectName(u"verticalLayout_10")
        self.verticalLayout_10.setContentsMargins(0, 0, 0, 0)
        self.headerContainer = QWidget(self.mainBodyContainer)
        self.headerContainer.setObjectName(u"headerContainer")
        self.horizontalLayout_6 = QHBoxLayout(self.headerContainer)
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.frame_5 = QFrame(self.headerContainer)
        self.frame_5.setObjectName(u"frame_5")
        self.frame_5.setFrameShape(QFrame.StyledPanel)
        self.frame_5.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_8 = QHBoxLayout(self.frame_5)
        self.horizontalLayout_8.setSpacing(6)
        self.horizontalLayout_8.setObjectName(u"horizontalLayout_8")
        self.horizontalLayout_8.setContentsMargins(0, 0, 0, 0)
        self.label_6 = QLabel(self.frame_5)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setMaximumSize(QSize(33, 36))
        self.label_6.setPixmap(QPixmap(u":/images/logo/googeese_logo2.png"))
        self.label_6.setScaledContents(True)

        self.horizontalLayout_8.addWidget(self.label_6)

        self.label_7 = QLabel(self.frame_5)
        self.label_7.setObjectName(u"label_7")
        font2 = QFont()
        font2.setBold(True)
        font2.setWeight(75)
        self.label_7.setFont(font2)
        self.label_7.setCursor(QCursor(Qt.PointingHandCursor))
        self.label_7.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_8.addWidget(self.label_7)


        self.horizontalLayout_6.addWidget(self.frame_5, 0, Qt.AlignLeft)

        self.frame_6 = QFrame(self.headerContainer)
        self.frame_6.setObjectName(u"frame_6")
        self.frame_6.setFrameShape(QFrame.StyledPanel)
        self.frame_6.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_7 = QHBoxLayout(self.frame_6)
        self.horizontalLayout_7.setSpacing(6)
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.horizontalLayout_7.setContentsMargins(0, 0, 0, 0)
        self.notificationBtn = QPushButton(self.frame_6)
        self.notificationBtn.setObjectName(u"notificationBtn")
        icon7 = QIcon()
        icon7.addFile(u":/icons/icons/blueIcons/bell.svg", QSize(), QIcon.Normal, QIcon.Off)
        self.notificationBtn.setIcon(icon7)
        self.notificationBtn.setIconSize(QSize(24, 24))

        self.horizontalLayout_7.addWidget(self.notificationBtn)

        self.moreMenuBtn = QPushButton(self.frame_6)
        self.moreMenuBtn.setObjectName(u"moreMenuBtn")
        icon8 = QIcon()
        icon8.addFile(u":/icons/icons/blueIcons/more-horizontal.svg", QSize(), QIcon.Normal, QIcon.Off)
        self.moreMenuBtn.setIcon(icon8)
        self.moreMenuBtn.setIconSize(QSize(24, 24))

        self.horizontalLayout_7.addWidget(self.moreMenuBtn)

        self.profileMenuBtn = QPushButton(self.frame_6)
        self.profileMenuBtn.setObjectName(u"profileMenuBtn")
        icon9 = QIcon()
        icon9.addFile(u":/icons/icons/blueIcons/user.svg", QSize(), QIcon.Normal, QIcon.Off)
        self.profileMenuBtn.setIcon(icon9)
        self.profileMenuBtn.setIconSize(QSize(24, 24))

        self.horizontalLayout_7.addWidget(self.profileMenuBtn)


        self.horizontalLayout_6.addWidget(self.frame_6, 0, Qt.AlignHCenter)

        self.frame_7 = QFrame(self.headerContainer)
        self.frame_7.setObjectName(u"frame_7")
        self.frame_7.setFrameShape(QFrame.StyledPanel)
        self.frame_7.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_5 = QHBoxLayout(self.frame_7)
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.horizontalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.minimizeBtn = QPushButton(self.frame_7)
        self.minimizeBtn.setObjectName(u"minimizeBtn")
        icon10 = QIcon()
        icon10.addFile(u":/icons/icons/blueIcons/minus.svg", QSize(), QIcon.Normal, QIcon.Off)
        self.minimizeBtn.setIcon(icon10)

        self.horizontalLayout_5.addWidget(self.minimizeBtn)

        self.restoreBtn = QPushButton(self.frame_7)
        self.restoreBtn.setObjectName(u"restoreBtn")
        icon11 = QIcon()
        icon11.addFile(u":/icons/icons/blueIcons/square.svg", QSize(), QIcon.Normal, QIcon.Off)
        self.restoreBtn.setIcon(icon11)

        self.horizontalLayout_5.addWidget(self.restoreBtn)

        self.closeBtn = QPushButton(self.frame_7)
        self.closeBtn.setObjectName(u"closeBtn")
        icon12 = QIcon()
        icon12.addFile(u":/icons/icons/blueIcons/x.svg", QSize(), QIcon.Normal, QIcon.Off)
        self.closeBtn.setIcon(icon12)

        self.horizontalLayout_5.addWidget(self.closeBtn)


        self.horizontalLayout_6.addWidget(self.frame_7, 0, Qt.AlignRight)


        self.verticalLayout_10.addWidget(self.headerContainer, 0, Qt.AlignTop)

        self.mainBodyContent = QWidget(self.mainBodyContainer)
        self.mainBodyContent.setObjectName(u"mainBodyContent")
        sizePolicy3 = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        sizePolicy3.setHorizontalStretch(0)
        sizePolicy3.setVerticalStretch(0)
        sizePolicy3.setHeightForWidth(self.mainBodyContent.sizePolicy().hasHeightForWidth())
        self.mainBodyContent.setSizePolicy(sizePolicy3)
        self.horizontalLayout_3 = QHBoxLayout(self.mainBodyContent)
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.mainContentsContainer = QWidget(self.mainBodyContent)
        self.mainContentsContainer.setObjectName(u"mainContentsContainer")
        self.verticalLayout_15 = QVBoxLayout(self.mainContentsContainer)
        self.verticalLayout_15.setObjectName(u"verticalLayout_15")
        self.verticalLayout_15.setContentsMargins(10, 10, 10, 0)
        self.mainPages = QCustomStackedWidget(self.mainContentsContainer)
        self.mainPages.setObjectName(u"mainPages")
        self.page_6 = QWidget()
        self.page_6.setObjectName(u"page_6")
        self.verticalLayout_16 = QVBoxLayout(self.page_6)
        self.verticalLayout_16.setSpacing(10)
        self.verticalLayout_16.setObjectName(u"verticalLayout_16")
        self.verticalLayout_16.setContentsMargins(0, 0, 0, 0)
        self.frame_13 = QFrame(self.page_6)
        self.frame_13.setObjectName(u"frame_13")
        self.frame_13.setFrameShape(QFrame.StyledPanel)
        self.frame_13.setFrameShadow(QFrame.Raised)
        self.verticalLayout_22 = QVBoxLayout(self.frame_13)
        self.verticalLayout_22.setSpacing(20)
        self.verticalLayout_22.setObjectName(u"verticalLayout_22")
        self.frame_14 = QFrame(self.frame_13)
        self.frame_14.setObjectName(u"frame_14")
        sizePolicy4 = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        sizePolicy4.setHorizontalStretch(0)
        sizePolicy4.setVerticalStretch(0)
        sizePolicy4.setHeightForWidth(self.frame_14.sizePolicy().hasHeightForWidth())
        self.frame_14.setSizePolicy(sizePolicy4)
        self.frame_14.setMinimumSize(QSize(0, 0))
        self.frame_14.setFrameShape(QFrame.StyledPanel)
        self.frame_14.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_16 = QHBoxLayout(self.frame_14)
        self.horizontalLayout_16.setObjectName(u"horizontalLayout_16")
        self.label_12 = QLabel(self.frame_14)
        self.label_12.setObjectName(u"label_12")
        sizePolicy5 = QSizePolicy(QSizePolicy.Maximum, QSizePolicy.Maximum)
        sizePolicy5.setHorizontalStretch(0)
        sizePolicy5.setVerticalStretch(0)
        sizePolicy5.setHeightForWidth(self.label_12.sizePolicy().hasHeightForWidth())
        self.label_12.setSizePolicy(sizePolicy5)
        self.label_12.setMinimumSize(QSize(588, 258))

        self.horizontalLayout_16.addWidget(self.label_12, 0, Qt.AlignHCenter|Qt.AlignVCenter)


        self.verticalLayout_22.addWidget(self.frame_14)

        self.frame_15 = QFrame(self.frame_13)
        self.frame_15.setObjectName(u"frame_15")
        sizePolicy4.setHeightForWidth(self.frame_15.sizePolicy().hasHeightForWidth())
        self.frame_15.setSizePolicy(sizePolicy4)
        self.frame_15.setFrameShape(QFrame.StyledPanel)
        self.frame_15.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_15 = QHBoxLayout(self.frame_15)
        self.horizontalLayout_15.setObjectName(u"horizontalLayout_15")
        self.card_1 = QFrame(self.frame_15)
        self.card_1.setObjectName(u"card_1")
        sizePolicy.setHeightForWidth(self.card_1.sizePolicy().hasHeightForWidth())
        self.card_1.setSizePolicy(sizePolicy)
        self.card_1.setMinimumSize(QSize(120, 0))
        self.card_1.setFrameShape(QFrame.StyledPanel)
        self.card_1.setFrameShadow(QFrame.Raised)
        self.verticalLayout_23 = QVBoxLayout(self.card_1)
        self.verticalLayout_23.setSpacing(0)
        self.verticalLayout_23.setObjectName(u"verticalLayout_23")
        self.verticalLayout_23.setContentsMargins(0, 0, 0, 0)
        self.frame_32 = QFrame(self.card_1)
        self.frame_32.setObjectName(u"frame_32")
        self.frame_32.setFrameShape(QFrame.StyledPanel)
        self.frame_32.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_21 = QHBoxLayout(self.frame_32)
        self.horizontalLayout_21.setObjectName(u"horizontalLayout_21")
        self.label_30 = QLabel(self.frame_32)
        self.label_30.setObjectName(u"label_30")
        font3 = QFont()
        font3.setPointSize(15)
        font3.setBold(True)
        font3.setWeight(75)
        self.label_30.setFont(font3)
        self.label_30.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_21.addWidget(self.label_30)


        self.verticalLayout_23.addWidget(self.frame_32, 0, Qt.AlignTop)

        self.frame_31 = QFrame(self.card_1)
        self.frame_31.setObjectName(u"frame_31")
        sizePolicy.setHeightForWidth(self.frame_31.sizePolicy().hasHeightForWidth())
        self.frame_31.setSizePolicy(sizePolicy)
        self.frame_31.setFrameShape(QFrame.StyledPanel)
        self.frame_31.setFrameShadow(QFrame.Raised)
        self.verticalLayout_26 = QVBoxLayout(self.frame_31)
        self.verticalLayout_26.setSpacing(0)
        self.verticalLayout_26.setObjectName(u"verticalLayout_26")
        self.verticalLayout_26.setContentsMargins(0, 0, 0, 10)
        self.frame_18 = QFrame(self.frame_31)
        self.frame_18.setObjectName(u"frame_18")
        self.frame_18.setFrameShape(QFrame.StyledPanel)
        self.frame_18.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_23 = QHBoxLayout(self.frame_18)
        self.horizontalLayout_23.setObjectName(u"horizontalLayout_23")
        self.label_status_b1 = QLabel(self.frame_18)
        self.label_status_b1.setObjectName(u"label_status_b1")
        self.label_status_b1.setMinimumSize(QSize(20, 20))
        self.label_status_b1.setMaximumSize(QSize(30, 30))
        font4 = QFont()
        font4.setPointSize(13)
        font4.setBold(True)
        font4.setWeight(75)
        self.label_status_b1.setFont(font4)
        self.label_status_b1.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_23.addWidget(self.label_status_b1, 0, Qt.AlignHCenter|Qt.AlignVCenter)


        self.verticalLayout_26.addWidget(self.frame_18, 0, Qt.AlignHCenter|Qt.AlignVCenter)

        self.frame_20 = QFrame(self.frame_31)
        self.frame_20.setObjectName(u"frame_20")
        sizePolicy.setHeightForWidth(self.frame_20.sizePolicy().hasHeightForWidth())
        self.frame_20.setSizePolicy(sizePolicy)
        self.frame_20.setMinimumSize(QSize(0, 84))
        self.frame_20.setFrameShape(QFrame.StyledPanel)
        self.frame_20.setFrameShadow(QFrame.Raised)
        self.verticalLayout_27 = QVBoxLayout(self.frame_20)
        self.verticalLayout_27.setObjectName(u"verticalLayout_27")
        self.verticalLayout_27.setContentsMargins(5, -1, 5, 5)
        self.frame_mt_b1 = QFrame(self.frame_20)
        self.frame_mt_b1.setObjectName(u"frame_mt_b1")
        sizePolicy1.setHeightForWidth(self.frame_mt_b1.sizePolicy().hasHeightForWidth())
        self.frame_mt_b1.setSizePolicy(sizePolicy1)
        self.frame_mt_b1.setCursor(QCursor(Qt.ArrowCursor))
        self.frame_mt_b1.setFrameShape(QFrame.StyledPanel)
        self.frame_mt_b1.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_27 = QHBoxLayout(self.frame_mt_b1)
        self.horizontalLayout_27.setSpacing(6)
        self.horizontalLayout_27.setObjectName(u"horizontalLayout_27")
        self.horizontalLayout_27.setContentsMargins(-1, 4, -1, 9)
        self.label_21 = QLabel(self.frame_mt_b1)
        self.label_21.setObjectName(u"label_21")
        self.label_21.setFont(font)

        self.horizontalLayout_27.addWidget(self.label_21, 0, Qt.AlignLeft|Qt.AlignVCenter)

        self.label_mt_b1 = QLabel(self.frame_mt_b1)
        self.label_mt_b1.setObjectName(u"label_mt_b1")
        self.label_mt_b1.setMinimumSize(QSize(20, 30))
        self.label_mt_b1.setMaximumSize(QSize(30, 40))
        self.label_mt_b1.setPixmap(QPixmap(u":/icons/icons/whiteIcons/engine-motor.svg"))
        self.label_mt_b1.setScaledContents(True)
        self.label_mt_b1.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_27.addWidget(self.label_mt_b1, 0, Qt.AlignRight|Qt.AlignVCenter)


        self.verticalLayout_27.addWidget(self.frame_mt_b1, 0, Qt.AlignVCenter)

        self.frame_li_b1 = QFrame(self.frame_20)
        self.frame_li_b1.setObjectName(u"frame_li_b1")
        sizePolicy1.setHeightForWidth(self.frame_li_b1.sizePolicy().hasHeightForWidth())
        self.frame_li_b1.setSizePolicy(sizePolicy1)
        self.frame_li_b1.setFrameShape(QFrame.StyledPanel)
        self.frame_li_b1.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_28 = QHBoxLayout(self.frame_li_b1)
        self.horizontalLayout_28.setObjectName(u"horizontalLayout_28")
        self.label_23 = QLabel(self.frame_li_b1)
        self.label_23.setObjectName(u"label_23")
        self.label_23.setFont(font)
        self.label_23.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_28.addWidget(self.label_23, 0, Qt.AlignLeft|Qt.AlignVCenter)

        self.label_li_b1 = QLabel(self.frame_li_b1)
        self.label_li_b1.setObjectName(u"label_li_b1")
        self.label_li_b1.setMinimumSize(QSize(20, 20))
        self.label_li_b1.setMaximumSize(QSize(30, 30))
        self.label_li_b1.setPixmap(QPixmap(u":/icons/icons/whiteIcons/radio.svg"))
        self.label_li_b1.setScaledContents(False)

        self.horizontalLayout_28.addWidget(self.label_li_b1, 0, Qt.AlignRight|Qt.AlignVCenter)


        self.verticalLayout_27.addWidget(self.frame_li_b1, 0, Qt.AlignVCenter)

        self.frame_mode_state_b1 = QFrame(self.frame_20)
        self.frame_mode_state_b1.setObjectName(u"frame_mode_state_b1")
        self.frame_mode_state_b1.setMinimumSize(QSize(0, 22))
        self.frame_mode_state_b1.setFrameShape(QFrame.StyledPanel)
        self.frame_mode_state_b1.setFrameShadow(QFrame.Raised)
        self.verticalLayout_17 = QVBoxLayout(self.frame_mode_state_b1)
        self.verticalLayout_17.setSpacing(0)
        self.verticalLayout_17.setObjectName(u"verticalLayout_17")
        self.verticalLayout_17.setContentsMargins(0, 0, 0, 0)
        self.label_mode_state_b1 = QLabel(self.frame_mode_state_b1)
        self.label_mode_state_b1.setObjectName(u"label_mode_state_b1")
        self.label_mode_state_b1.setMinimumSize(QSize(0, 22))
        self.label_mode_state_b1.setAlignment(Qt.AlignLeading|Qt.AlignLeft|Qt.AlignVCenter)

        self.verticalLayout_17.addWidget(self.label_mode_state_b1)


        self.verticalLayout_27.addWidget(self.frame_mode_state_b1)


        self.verticalLayout_26.addWidget(self.frame_20, 0, Qt.AlignHCenter|Qt.AlignVCenter)


        self.verticalLayout_23.addWidget(self.frame_31, 0, Qt.AlignHCenter|Qt.AlignVCenter)


        self.horizontalLayout_15.addWidget(self.card_1)

        self.card_2 = QFrame(self.frame_15)
        self.card_2.setObjectName(u"card_2")
        self.card_2.setMinimumSize(QSize(120, 0))
        self.card_2.setFrameShape(QFrame.StyledPanel)
        self.card_2.setFrameShadow(QFrame.Raised)
        self.verticalLayout_24 = QVBoxLayout(self.card_2)
        self.verticalLayout_24.setSpacing(0)
        self.verticalLayout_24.setObjectName(u"verticalLayout_24")
        self.verticalLayout_24.setContentsMargins(0, 0, 0, 0)
        self.frame_34 = QFrame(self.card_2)
        self.frame_34.setObjectName(u"frame_34")
        self.frame_34.setFrameShape(QFrame.StyledPanel)
        self.frame_34.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_22 = QHBoxLayout(self.frame_34)
        self.horizontalLayout_22.setObjectName(u"horizontalLayout_22")
        self.label_34 = QLabel(self.frame_34)
        self.label_34.setObjectName(u"label_34")
        self.label_34.setFont(font3)
        self.label_34.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_22.addWidget(self.label_34)


        self.verticalLayout_24.addWidget(self.frame_34)

        self.frame_33 = QFrame(self.card_2)
        self.frame_33.setObjectName(u"frame_33")
        sizePolicy.setHeightForWidth(self.frame_33.sizePolicy().hasHeightForWidth())
        self.frame_33.setSizePolicy(sizePolicy)
        self.frame_33.setFrameShape(QFrame.StyledPanel)
        self.frame_33.setFrameShadow(QFrame.Raised)
        self.verticalLayout_29 = QVBoxLayout(self.frame_33)
        self.verticalLayout_29.setSpacing(0)
        self.verticalLayout_29.setObjectName(u"verticalLayout_29")
        self.verticalLayout_29.setContentsMargins(0, 0, 0, 10)
        self.frame_21 = QFrame(self.frame_33)
        self.frame_21.setObjectName(u"frame_21")
        self.frame_21.setFrameShape(QFrame.StyledPanel)
        self.frame_21.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_24 = QHBoxLayout(self.frame_21)
        self.horizontalLayout_24.setObjectName(u"horizontalLayout_24")
        self.label_status_b2 = QLabel(self.frame_21)
        self.label_status_b2.setObjectName(u"label_status_b2")
        self.label_status_b2.setMinimumSize(QSize(20, 20))
        self.label_status_b2.setMaximumSize(QSize(30, 30))
        self.label_status_b2.setFont(font4)
        self.label_status_b2.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_24.addWidget(self.label_status_b2, 0, Qt.AlignHCenter|Qt.AlignVCenter)


        self.verticalLayout_29.addWidget(self.frame_21, 0, Qt.AlignHCenter|Qt.AlignVCenter)

        self.frame_22 = QFrame(self.frame_33)
        self.frame_22.setObjectName(u"frame_22")
        sizePolicy.setHeightForWidth(self.frame_22.sizePolicy().hasHeightForWidth())
        self.frame_22.setSizePolicy(sizePolicy)
        self.frame_22.setMinimumSize(QSize(0, 84))
        self.frame_22.setFrameShape(QFrame.StyledPanel)
        self.frame_22.setFrameShadow(QFrame.Raised)
        self.verticalLayout_31 = QVBoxLayout(self.frame_22)
        self.verticalLayout_31.setObjectName(u"verticalLayout_31")
        self.verticalLayout_31.setContentsMargins(5, -1, 5, 5)
        self.frame_mt_b2 = QFrame(self.frame_22)
        self.frame_mt_b2.setObjectName(u"frame_mt_b2")
        sizePolicy1.setHeightForWidth(self.frame_mt_b2.sizePolicy().hasHeightForWidth())
        self.frame_mt_b2.setSizePolicy(sizePolicy1)
        self.frame_mt_b2.setCursor(QCursor(Qt.ArrowCursor))
        self.frame_mt_b2.setFrameShape(QFrame.StyledPanel)
        self.frame_mt_b2.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_29 = QHBoxLayout(self.frame_mt_b2)
        self.horizontalLayout_29.setObjectName(u"horizontalLayout_29")
        self.horizontalLayout_29.setContentsMargins(-1, 4, -1, -1)
        self.label_26 = QLabel(self.frame_mt_b2)
        self.label_26.setObjectName(u"label_26")
        self.label_26.setFont(font)

        self.horizontalLayout_29.addWidget(self.label_26, 0, Qt.AlignLeft|Qt.AlignVCenter)

        self.label_mt_b2 = QLabel(self.frame_mt_b2)
        self.label_mt_b2.setObjectName(u"label_mt_b2")
        self.label_mt_b2.setMinimumSize(QSize(20, 30))
        self.label_mt_b2.setMaximumSize(QSize(30, 40))
        self.label_mt_b2.setPixmap(QPixmap(u":/icons/icons/whiteIcons/engine-motor.svg"))
        self.label_mt_b2.setScaledContents(True)
        self.label_mt_b2.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_29.addWidget(self.label_mt_b2, 0, Qt.AlignRight)


        self.verticalLayout_31.addWidget(self.frame_mt_b2, 0, Qt.AlignVCenter)

        self.frame_li_b2 = QFrame(self.frame_22)
        self.frame_li_b2.setObjectName(u"frame_li_b2")
        sizePolicy1.setHeightForWidth(self.frame_li_b2.sizePolicy().hasHeightForWidth())
        self.frame_li_b2.setSizePolicy(sizePolicy1)
        self.frame_li_b2.setFrameShape(QFrame.StyledPanel)
        self.frame_li_b2.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_30 = QHBoxLayout(self.frame_li_b2)
        self.horizontalLayout_30.setObjectName(u"horizontalLayout_30")
        self.label_28 = QLabel(self.frame_li_b2)
        self.label_28.setObjectName(u"label_28")
        self.label_28.setFont(font)
        self.label_28.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_30.addWidget(self.label_28, 0, Qt.AlignLeft|Qt.AlignVCenter)

        self.label_li_b2 = QLabel(self.frame_li_b2)
        self.label_li_b2.setObjectName(u"label_li_b2")
        self.label_li_b2.setMinimumSize(QSize(20, 20))
        self.label_li_b2.setMaximumSize(QSize(30, 30))
        self.label_li_b2.setPixmap(QPixmap(u":/icons/icons/whiteIcons/radio.svg"))
        self.label_li_b2.setScaledContents(False)

        self.horizontalLayout_30.addWidget(self.label_li_b2, 0, Qt.AlignRight|Qt.AlignVCenter)


        self.verticalLayout_31.addWidget(self.frame_li_b2, 0, Qt.AlignVCenter)

        self.frame_mode_state_b2 = QFrame(self.frame_22)
        self.frame_mode_state_b2.setObjectName(u"frame_mode_state_b2")
        self.frame_mode_state_b2.setMinimumSize(QSize(0, 22))
        self.frame_mode_state_b2.setFrameShape(QFrame.StyledPanel)
        self.frame_mode_state_b2.setFrameShadow(QFrame.Raised)
        self.verticalLayout_21 = QVBoxLayout(self.frame_mode_state_b2)
        self.verticalLayout_21.setSpacing(0)
        self.verticalLayout_21.setObjectName(u"verticalLayout_21")
        self.verticalLayout_21.setContentsMargins(0, 0, 0, 0)
        self.label_mode_state_b2 = QLabel(self.frame_mode_state_b2)
        self.label_mode_state_b2.setObjectName(u"label_mode_state_b2")
        self.label_mode_state_b2.setMinimumSize(QSize(0, 22))
        self.label_mode_state_b2.setAlignment(Qt.AlignCenter)

        self.verticalLayout_21.addWidget(self.label_mode_state_b2)


        self.verticalLayout_31.addWidget(self.frame_mode_state_b2)


        self.verticalLayout_29.addWidget(self.frame_22, 0, Qt.AlignHCenter|Qt.AlignVCenter)


        self.verticalLayout_24.addWidget(self.frame_33, 0, Qt.AlignHCenter|Qt.AlignVCenter)


        self.horizontalLayout_15.addWidget(self.card_2)

        self.card_3 = QFrame(self.frame_15)
        self.card_3.setObjectName(u"card_3")
        self.card_3.setMinimumSize(QSize(120, 0))
        self.card_3.setFrameShape(QFrame.StyledPanel)
        self.card_3.setFrameShadow(QFrame.Raised)
        self.verticalLayout_28 = QVBoxLayout(self.card_3)
        self.verticalLayout_28.setSpacing(0)
        self.verticalLayout_28.setObjectName(u"verticalLayout_28")
        self.verticalLayout_28.setContentsMargins(0, 0, 0, 0)
        self.frame_27 = QFrame(self.card_3)
        self.frame_27.setObjectName(u"frame_27")
        self.frame_27.setFrameShape(QFrame.StyledPanel)
        self.frame_27.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_19 = QHBoxLayout(self.frame_27)
        self.horizontalLayout_19.setObjectName(u"horizontalLayout_19")
        self.label_22 = QLabel(self.frame_27)
        self.label_22.setObjectName(u"label_22")
        self.label_22.setFont(font3)
        self.label_22.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_19.addWidget(self.label_22)


        self.verticalLayout_28.addWidget(self.frame_27)

        self.frame_35 = QFrame(self.card_3)
        self.frame_35.setObjectName(u"frame_35")
        sizePolicy.setHeightForWidth(self.frame_35.sizePolicy().hasHeightForWidth())
        self.frame_35.setSizePolicy(sizePolicy)
        self.frame_35.setFrameShape(QFrame.StyledPanel)
        self.frame_35.setFrameShadow(QFrame.Raised)
        self.verticalLayout_32 = QVBoxLayout(self.frame_35)
        self.verticalLayout_32.setSpacing(0)
        self.verticalLayout_32.setObjectName(u"verticalLayout_32")
        self.verticalLayout_32.setContentsMargins(0, 0, 0, 10)
        self.frame_23 = QFrame(self.frame_35)
        self.frame_23.setObjectName(u"frame_23")
        self.frame_23.setFrameShape(QFrame.StyledPanel)
        self.frame_23.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_25 = QHBoxLayout(self.frame_23)
        self.horizontalLayout_25.setObjectName(u"horizontalLayout_25")
        self.label_status_b3 = QLabel(self.frame_23)
        self.label_status_b3.setObjectName(u"label_status_b3")
        self.label_status_b3.setMinimumSize(QSize(20, 20))
        self.label_status_b3.setMaximumSize(QSize(30, 30))
        self.label_status_b3.setFont(font4)
        self.label_status_b3.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_25.addWidget(self.label_status_b3, 0, Qt.AlignHCenter|Qt.AlignVCenter)


        self.verticalLayout_32.addWidget(self.frame_23, 0, Qt.AlignHCenter|Qt.AlignVCenter)

        self.frame_24 = QFrame(self.frame_35)
        self.frame_24.setObjectName(u"frame_24")
        sizePolicy.setHeightForWidth(self.frame_24.sizePolicy().hasHeightForWidth())
        self.frame_24.setSizePolicy(sizePolicy)
        self.frame_24.setMinimumSize(QSize(0, 84))
        self.frame_24.setFrameShape(QFrame.StyledPanel)
        self.frame_24.setFrameShadow(QFrame.Raised)
        self.verticalLayout_33 = QVBoxLayout(self.frame_24)
        self.verticalLayout_33.setObjectName(u"verticalLayout_33")
        self.verticalLayout_33.setContentsMargins(5, -1, 5, 5)
        self.frame_mt_b3 = QFrame(self.frame_24)
        self.frame_mt_b3.setObjectName(u"frame_mt_b3")
        sizePolicy1.setHeightForWidth(self.frame_mt_b3.sizePolicy().hasHeightForWidth())
        self.frame_mt_b3.setSizePolicy(sizePolicy1)
        self.frame_mt_b3.setCursor(QCursor(Qt.ArrowCursor))
        self.frame_mt_b3.setFrameShape(QFrame.StyledPanel)
        self.frame_mt_b3.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_31 = QHBoxLayout(self.frame_mt_b3)
        self.horizontalLayout_31.setObjectName(u"horizontalLayout_31")
        self.horizontalLayout_31.setContentsMargins(-1, 4, -1, -1)
        self.label_27 = QLabel(self.frame_mt_b3)
        self.label_27.setObjectName(u"label_27")
        self.label_27.setFont(font)

        self.horizontalLayout_31.addWidget(self.label_27, 0, Qt.AlignLeft|Qt.AlignVCenter)

        self.label_mt_b3 = QLabel(self.frame_mt_b3)
        self.label_mt_b3.setObjectName(u"label_mt_b3")
        self.label_mt_b3.setMinimumSize(QSize(20, 30))
        self.label_mt_b3.setMaximumSize(QSize(30, 40))
        self.label_mt_b3.setPixmap(QPixmap(u":/icons/icons/whiteIcons/engine-motor.svg"))
        self.label_mt_b3.setScaledContents(True)
        self.label_mt_b3.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_31.addWidget(self.label_mt_b3, 0, Qt.AlignTop)


        self.verticalLayout_33.addWidget(self.frame_mt_b3, 0, Qt.AlignVCenter)

        self.frame_li_b3 = QFrame(self.frame_24)
        self.frame_li_b3.setObjectName(u"frame_li_b3")
        sizePolicy1.setHeightForWidth(self.frame_li_b3.sizePolicy().hasHeightForWidth())
        self.frame_li_b3.setSizePolicy(sizePolicy1)
        self.frame_li_b3.setFrameShape(QFrame.StyledPanel)
        self.frame_li_b3.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_32 = QHBoxLayout(self.frame_li_b3)
        self.horizontalLayout_32.setObjectName(u"horizontalLayout_32")
        self.label_29 = QLabel(self.frame_li_b3)
        self.label_29.setObjectName(u"label_29")
        self.label_29.setFont(font)
        self.label_29.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_32.addWidget(self.label_29, 0, Qt.AlignLeft|Qt.AlignVCenter)

        self.label_li_b3 = QLabel(self.frame_li_b3)
        self.label_li_b3.setObjectName(u"label_li_b3")
        self.label_li_b3.setMinimumSize(QSize(20, 20))
        self.label_li_b3.setMaximumSize(QSize(30, 30))
        self.label_li_b3.setPixmap(QPixmap(u":/icons/icons/whiteIcons/radio.svg"))
        self.label_li_b3.setScaledContents(False)

        self.horizontalLayout_32.addWidget(self.label_li_b3, 0, Qt.AlignHCenter|Qt.AlignVCenter)


        self.verticalLayout_33.addWidget(self.frame_li_b3, 0, Qt.AlignVCenter)

        self.frame_mode_state_b3 = QFrame(self.frame_24)
        self.frame_mode_state_b3.setObjectName(u"frame_mode_state_b3")
        self.frame_mode_state_b3.setMinimumSize(QSize(0, 22))
        self.frame_mode_state_b3.setFrameShape(QFrame.StyledPanel)
        self.frame_mode_state_b3.setFrameShadow(QFrame.Raised)
        self.verticalLayout_25 = QVBoxLayout(self.frame_mode_state_b3)
        self.verticalLayout_25.setObjectName(u"verticalLayout_25")
        self.label_mode_state_b3 = QLabel(self.frame_mode_state_b3)
        self.label_mode_state_b3.setObjectName(u"label_mode_state_b3")
        self.label_mode_state_b3.setMinimumSize(QSize(0, 22))

        self.verticalLayout_25.addWidget(self.label_mode_state_b3)


        self.verticalLayout_33.addWidget(self.frame_mode_state_b3)


        self.verticalLayout_32.addWidget(self.frame_24, 0, Qt.AlignHCenter|Qt.AlignVCenter)


        self.verticalLayout_28.addWidget(self.frame_35)


        self.horizontalLayout_15.addWidget(self.card_3)

        self.card_4 = QFrame(self.frame_15)
        self.card_4.setObjectName(u"card_4")
        self.card_4.setMinimumSize(QSize(120, 0))
        self.card_4.setFrameShape(QFrame.StyledPanel)
        self.card_4.setFrameShadow(QFrame.Raised)
        self.verticalLayout_30 = QVBoxLayout(self.card_4)
        self.verticalLayout_30.setSpacing(0)
        self.verticalLayout_30.setObjectName(u"verticalLayout_30")
        self.verticalLayout_30.setContentsMargins(0, 0, 0, 0)
        self.frame_29 = QFrame(self.card_4)
        self.frame_29.setObjectName(u"frame_29")
        self.frame_29.setFrameShape(QFrame.StyledPanel)
        self.frame_29.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_20 = QHBoxLayout(self.frame_29)
        self.horizontalLayout_20.setObjectName(u"horizontalLayout_20")
        self.label_25 = QLabel(self.frame_29)
        self.label_25.setObjectName(u"label_25")
        self.label_25.setFont(font3)
        self.label_25.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_20.addWidget(self.label_25)


        self.verticalLayout_30.addWidget(self.frame_29)

        self.frame_36 = QFrame(self.card_4)
        self.frame_36.setObjectName(u"frame_36")
        sizePolicy.setHeightForWidth(self.frame_36.sizePolicy().hasHeightForWidth())
        self.frame_36.setSizePolicy(sizePolicy)
        self.frame_36.setFrameShape(QFrame.StyledPanel)
        self.frame_36.setFrameShadow(QFrame.Raised)
        self.verticalLayout_34 = QVBoxLayout(self.frame_36)
        self.verticalLayout_34.setSpacing(0)
        self.verticalLayout_34.setObjectName(u"verticalLayout_34")
        self.verticalLayout_34.setContentsMargins(0, 0, 0, 10)
        self.frame_25 = QFrame(self.frame_36)
        self.frame_25.setObjectName(u"frame_25")
        self.frame_25.setFrameShape(QFrame.StyledPanel)
        self.frame_25.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_26 = QHBoxLayout(self.frame_25)
        self.horizontalLayout_26.setObjectName(u"horizontalLayout_26")
        self.label_status_b4 = QLabel(self.frame_25)
        self.label_status_b4.setObjectName(u"label_status_b4")
        self.label_status_b4.setMinimumSize(QSize(20, 20))
        self.label_status_b4.setMaximumSize(QSize(30, 30))
        self.label_status_b4.setFont(font4)
        self.label_status_b4.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_26.addWidget(self.label_status_b4, 0, Qt.AlignHCenter|Qt.AlignVCenter)


        self.verticalLayout_34.addWidget(self.frame_25, 0, Qt.AlignHCenter|Qt.AlignVCenter)

        self.frame_26 = QFrame(self.frame_36)
        self.frame_26.setObjectName(u"frame_26")
        sizePolicy.setHeightForWidth(self.frame_26.sizePolicy().hasHeightForWidth())
        self.frame_26.setSizePolicy(sizePolicy)
        self.frame_26.setMinimumSize(QSize(0, 84))
        self.frame_26.setFrameShape(QFrame.StyledPanel)
        self.frame_26.setFrameShadow(QFrame.Raised)
        self.verticalLayout_35 = QVBoxLayout(self.frame_26)
        self.verticalLayout_35.setObjectName(u"verticalLayout_35")
        self.verticalLayout_35.setContentsMargins(5, -1, 5, 5)
        self.frame_mt_b4 = QFrame(self.frame_26)
        self.frame_mt_b4.setObjectName(u"frame_mt_b4")
        sizePolicy1.setHeightForWidth(self.frame_mt_b4.sizePolicy().hasHeightForWidth())
        self.frame_mt_b4.setSizePolicy(sizePolicy1)
        self.frame_mt_b4.setCursor(QCursor(Qt.ArrowCursor))
        self.frame_mt_b4.setFrameShape(QFrame.StyledPanel)
        self.frame_mt_b4.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_33 = QHBoxLayout(self.frame_mt_b4)
        self.horizontalLayout_33.setObjectName(u"horizontalLayout_33")
        self.horizontalLayout_33.setContentsMargins(-1, 4, -1, -1)
        self.label_31 = QLabel(self.frame_mt_b4)
        self.label_31.setObjectName(u"label_31")
        self.label_31.setFont(font)

        self.horizontalLayout_33.addWidget(self.label_31, 0, Qt.AlignLeft|Qt.AlignVCenter)

        self.label_mt_b4 = QLabel(self.frame_mt_b4)
        self.label_mt_b4.setObjectName(u"label_mt_b4")
        self.label_mt_b4.setMinimumSize(QSize(20, 30))
        self.label_mt_b4.setMaximumSize(QSize(30, 40))
        self.label_mt_b4.setPixmap(QPixmap(u":/icons/icons/whiteIcons/engine-motor.svg"))
        self.label_mt_b4.setScaledContents(True)
        self.label_mt_b4.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_33.addWidget(self.label_mt_b4, 0, Qt.AlignRight)


        self.verticalLayout_35.addWidget(self.frame_mt_b4, 0, Qt.AlignVCenter)

        self.frame_li_b4 = QFrame(self.frame_26)
        self.frame_li_b4.setObjectName(u"frame_li_b4")
        sizePolicy1.setHeightForWidth(self.frame_li_b4.sizePolicy().hasHeightForWidth())
        self.frame_li_b4.setSizePolicy(sizePolicy1)
        self.frame_li_b4.setFrameShape(QFrame.StyledPanel)
        self.frame_li_b4.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_34 = QHBoxLayout(self.frame_li_b4)
        self.horizontalLayout_34.setObjectName(u"horizontalLayout_34")
        self.label_32 = QLabel(self.frame_li_b4)
        self.label_32.setObjectName(u"label_32")
        self.label_32.setFont(font)
        self.label_32.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_34.addWidget(self.label_32, 0, Qt.AlignLeft|Qt.AlignVCenter)

        self.label_li_b4 = QLabel(self.frame_li_b4)
        self.label_li_b4.setObjectName(u"label_li_b4")
        self.label_li_b4.setMinimumSize(QSize(20, 20))
        self.label_li_b4.setMaximumSize(QSize(30, 30))
        self.label_li_b4.setPixmap(QPixmap(u":/icons/icons/whiteIcons/radio.svg"))
        self.label_li_b4.setScaledContents(False)

        self.horizontalLayout_34.addWidget(self.label_li_b4, 0, Qt.AlignHCenter|Qt.AlignVCenter)


        self.verticalLayout_35.addWidget(self.frame_li_b4, 0, Qt.AlignVCenter)

        self.frame_mode_state_b4 = QFrame(self.frame_26)
        self.frame_mode_state_b4.setObjectName(u"frame_mode_state_b4")
        self.frame_mode_state_b4.setMinimumSize(QSize(0, 22))
        self.frame_mode_state_b4.setFrameShape(QFrame.StyledPanel)
        self.frame_mode_state_b4.setFrameShadow(QFrame.Raised)
        self.verticalLayout_36 = QVBoxLayout(self.frame_mode_state_b4)
        self.verticalLayout_36.setObjectName(u"verticalLayout_36")
        self.label_mode_state_b4 = QLabel(self.frame_mode_state_b4)
        self.label_mode_state_b4.setObjectName(u"label_mode_state_b4")
        self.label_mode_state_b4.setMinimumSize(QSize(0, 22))

        self.verticalLayout_36.addWidget(self.label_mode_state_b4)


        self.verticalLayout_35.addWidget(self.frame_mode_state_b4)


        self.verticalLayout_34.addWidget(self.frame_26, 0, Qt.AlignHCenter|Qt.AlignVCenter)


        self.verticalLayout_30.addWidget(self.frame_36)


        self.horizontalLayout_15.addWidget(self.card_4)


        self.verticalLayout_22.addWidget(self.frame_15)


        self.verticalLayout_16.addWidget(self.frame_13)

        self.mainPages.addWidget(self.page_6)
        self.page_8 = QWidget()
        self.page_8.setObjectName(u"page_8")
        self.verticalLayout_18 = QVBoxLayout(self.page_8)
        self.verticalLayout_18.setSpacing(5)
        self.verticalLayout_18.setObjectName(u"verticalLayout_18")
        self.verticalLayout_18.setContentsMargins(0, 0, 0, 0)
        self.gridLayout = QGridLayout()
        self.gridLayout.setSpacing(5)
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.videoLabel_TopLeft = QLabel(self.page_8)
        self.videoLabel_TopLeft.setObjectName(u"videoLabel_TopLeft")
        self.videoLabel_TopLeft.setMinimumSize(QSize(160, 120))
        self.videoLabel_TopLeft.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.videoLabel_TopLeft, 0, 0, 1, 1)

        self.videoLabel_TopRight = QLabel(self.page_8)
        self.videoLabel_TopRight.setObjectName(u"videoLabel_TopRight")
        self.videoLabel_TopRight.setMinimumSize(QSize(160, 120))
        self.videoLabel_TopRight.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.videoLabel_TopRight, 0, 1, 1, 1)

        self.videoLabel_BottomLeft = QLabel(self.page_8)
        self.videoLabel_BottomLeft.setObjectName(u"videoLabel_BottomLeft")
        self.videoLabel_BottomLeft.setMinimumSize(QSize(160, 120))
        self.videoLabel_BottomLeft.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.videoLabel_BottomLeft, 1, 0, 1, 1)

        self.videoLabel_BottomRight = QLabel(self.page_8)
        self.videoLabel_BottomRight.setObjectName(u"videoLabel_BottomRight")
        self.videoLabel_BottomRight.setMinimumSize(QSize(160, 120))
        self.videoLabel_BottomRight.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.videoLabel_BottomRight, 1, 1, 1, 1)


        self.verticalLayout_18.addLayout(self.gridLayout)

        self.frame_16 = QFrame(self.page_8)
        self.frame_16.setObjectName(u"frame_16")
        sizePolicy.setHeightForWidth(self.frame_16.sizePolicy().hasHeightForWidth())
        self.frame_16.setSizePolicy(sizePolicy)
        self.frame_16.setMaximumSize(QSize(450, 90))
        self.frame_16.setFrameShape(QFrame.StyledPanel)
        self.frame_16.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_17 = QHBoxLayout(self.frame_16)
        self.horizontalLayout_17.setObjectName(u"horizontalLayout_17")
        self.horizontalLayout_17.setContentsMargins(-1, 0, -1, 0)
        self.frame_17 = QFrame(self.frame_16)
        self.frame_17.setObjectName(u"frame_17")
        sizePolicy5.setHeightForWidth(self.frame_17.sizePolicy().hasHeightForWidth())
        self.frame_17.setSizePolicy(sizePolicy5)
        self.frame_17.setMaximumSize(QSize(16777215, 90))
        self.frame_17.setFrameShape(QFrame.StyledPanel)
        self.frame_17.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_18 = QHBoxLayout(self.frame_17)
        self.horizontalLayout_18.setObjectName(u"horizontalLayout_18")
        self.horizontalLayout_18.setContentsMargins(-1, 0, -1, 0)
        self.controlBtnLayout = QGridLayout()
        self.controlBtnLayout.setSpacing(0)
        self.controlBtnLayout.setObjectName(u"controlBtnLayout")
        self.label_16 = QLabel(self.frame_17)
        self.label_16.setObjectName(u"label_16")
        font5 = QFont()
        font5.setPointSize(10)
        font5.setBold(True)
        font5.setWeight(75)
        self.label_16.setFont(font5)
        self.label_16.setAlignment(Qt.AlignCenter)

        self.controlBtnLayout.addWidget(self.label_16, 0, 2, 1, 1)

        self.label_18 = QLabel(self.frame_17)
        self.label_18.setObjectName(u"label_18")
        self.label_18.setFont(font5)
        self.label_18.setAlignment(Qt.AlignCenter)

        self.controlBtnLayout.addWidget(self.label_18, 1, 0, 1, 1)

        self.emergencyBtn_b1 = QPushButton(self.frame_17)
        self.emergencyBtn_b1.setObjectName(u"emergencyBtn_b1")
        self.emergencyBtn_b1.setCursor(QCursor(Qt.PointingHandCursor))
        icon13 = QIcon()
        icon13.addFile(u":/icons/icons/blueIcons/stop-circle.svg", QSize(), QIcon.Normal, QIcon.Off)
        self.emergencyBtn_b1.setIcon(icon13)
        self.emergencyBtn_b1.setIconSize(QSize(35, 35))

        self.controlBtnLayout.addWidget(self.emergencyBtn_b1, 2, 0, 1, 1, Qt.AlignHCenter|Qt.AlignVCenter)

        self.label_11 = QLabel(self.frame_17)
        self.label_11.setObjectName(u"label_11")
        self.label_11.setFont(font5)
        self.label_11.setAlignment(Qt.AlignCenter)

        self.controlBtnLayout.addWidget(self.label_11, 0, 0, 1, 1)

        self.label_10 = QLabel(self.frame_17)
        self.label_10.setObjectName(u"label_10")
        self.label_10.setFont(font5)

        self.controlBtnLayout.addWidget(self.label_10, 1, 2, 1, 1)

        self.emergencyBtn_b2 = QPushButton(self.frame_17)
        self.emergencyBtn_b2.setObjectName(u"emergencyBtn_b2")
        self.emergencyBtn_b2.setCursor(QCursor(Qt.PointingHandCursor))
        self.emergencyBtn_b2.setIcon(icon13)
        self.emergencyBtn_b2.setIconSize(QSize(35, 35))

        self.controlBtnLayout.addWidget(self.emergencyBtn_b2, 2, 2, 1, 1, Qt.AlignHCenter|Qt.AlignVCenter)

        self.horizontalSpacer_2 = QSpacerItem(60, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.controlBtnLayout.addItem(self.horizontalSpacer_2, 0, 1, 1, 1)


        self.horizontalLayout_18.addLayout(self.controlBtnLayout)


        self.horizontalLayout_17.addWidget(self.frame_17, 0, Qt.AlignHCenter|Qt.AlignVCenter)


        self.verticalLayout_18.addWidget(self.frame_16, 0, Qt.AlignHCenter)

        self.mainPages.addWidget(self.page_8)

        self.verticalLayout_15.addWidget(self.mainPages)


        self.horizontalLayout_3.addWidget(self.mainContentsContainer)

        self.rightMenuContainer = QCustomSlideMenu(self.mainBodyContent)
        self.rightMenuContainer.setObjectName(u"rightMenuContainer")
        self.rightMenuContainer.setMinimumSize(QSize(200, 0))
        self.verticalLayout_11 = QVBoxLayout(self.rightMenuContainer)
        self.verticalLayout_11.setSpacing(0)
        self.verticalLayout_11.setObjectName(u"verticalLayout_11")
        self.verticalLayout_11.setContentsMargins(0, 0, 0, 0)
        self.rightMenuSubContainer = QWidget(self.rightMenuContainer)
        self.rightMenuSubContainer.setObjectName(u"rightMenuSubContainer")
        self.rightMenuSubContainer.setMinimumSize(QSize(200, 0))
        self.verticalLayout_12 = QVBoxLayout(self.rightMenuSubContainer)
        self.verticalLayout_12.setSpacing(5)
        self.verticalLayout_12.setObjectName(u"verticalLayout_12")
        self.verticalLayout_12.setContentsMargins(5, 5, 5, 5)
        self.frame_8 = QFrame(self.rightMenuSubContainer)
        self.frame_8.setObjectName(u"frame_8")
        self.frame_8.setFrameShape(QFrame.StyledPanel)
        self.frame_8.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_9 = QHBoxLayout(self.frame_8)
        self.horizontalLayout_9.setObjectName(u"horizontalLayout_9")
        self.label = QLabel(self.frame_8)
        self.label.setObjectName(u"label")
        self.label.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_9.addWidget(self.label)

        self.closeRightMenuBtn = QPushButton(self.frame_8)
        self.closeRightMenuBtn.setObjectName(u"closeRightMenuBtn")
        self.closeRightMenuBtn.setIcon(icon6)
        self.closeRightMenuBtn.setIconSize(QSize(24, 24))

        self.horizontalLayout_9.addWidget(self.closeRightMenuBtn, 0, Qt.AlignRight)


        self.verticalLayout_12.addWidget(self.frame_8)

        self.rightMenuPages = QCustomStackedWidget(self.rightMenuSubContainer)
        self.rightMenuPages.setObjectName(u"rightMenuPages")
        self.page_4 = QWidget()
        self.page_4.setObjectName(u"page_4")
        self.verticalLayout_13 = QVBoxLayout(self.page_4)
        self.verticalLayout_13.setObjectName(u"verticalLayout_13")
        self.label_8 = QLabel(self.page_4)
        self.label_8.setObjectName(u"label_8")
        self.label_8.setFont(font)
        self.label_8.setAlignment(Qt.AlignCenter)

        self.verticalLayout_13.addWidget(self.label_8)

        self.rightMenuPages.addWidget(self.page_4)
        self.page_5 = QWidget()
        self.page_5.setObjectName(u"page_5")
        self.verticalLayout_14 = QVBoxLayout(self.page_5)
        self.verticalLayout_14.setObjectName(u"verticalLayout_14")
        self.label_9 = QLabel(self.page_5)
        self.label_9.setObjectName(u"label_9")
        self.label_9.setFont(font)
        self.label_9.setAlignment(Qt.AlignCenter)

        self.verticalLayout_14.addWidget(self.label_9)

        self.rightMenuPages.addWidget(self.page_5)

        self.verticalLayout_12.addWidget(self.rightMenuPages)


        self.verticalLayout_11.addWidget(self.rightMenuSubContainer)


        self.horizontalLayout_3.addWidget(self.rightMenuContainer, 0, Qt.AlignRight)


        self.verticalLayout_10.addWidget(self.mainBodyContent)

        self.popupNotificationContainer = QCustomSlideMenu(self.mainBodyContainer)
        self.popupNotificationContainer.setObjectName(u"popupNotificationContainer")
        self.verticalLayout_19 = QVBoxLayout(self.popupNotificationContainer)
        self.verticalLayout_19.setObjectName(u"verticalLayout_19")
        self.popupNotificationSubContainer = QWidget(self.popupNotificationContainer)
        self.popupNotificationSubContainer.setObjectName(u"popupNotificationSubContainer")
        self.verticalLayout_20 = QVBoxLayout(self.popupNotificationSubContainer)
        self.verticalLayout_20.setObjectName(u"verticalLayout_20")
        self.label_14 = QLabel(self.popupNotificationSubContainer)
        self.label_14.setObjectName(u"label_14")
        self.label_14.setFont(font2)

        self.verticalLayout_20.addWidget(self.label_14)

        self.frame_9 = QFrame(self.popupNotificationSubContainer)
        self.frame_9.setObjectName(u"frame_9")
        self.frame_9.setFrameShape(QFrame.StyledPanel)
        self.frame_9.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_10 = QHBoxLayout(self.frame_9)
        self.horizontalLayout_10.setObjectName(u"horizontalLayout_10")
        self.label_13 = QLabel(self.frame_9)
        self.label_13.setObjectName(u"label_13")
        sizePolicy1.setHeightForWidth(self.label_13.sizePolicy().hasHeightForWidth())
        self.label_13.setSizePolicy(sizePolicy1)
        self.label_13.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_10.addWidget(self.label_13)

        self.closeNotificationBtn = QPushButton(self.frame_9)
        self.closeNotificationBtn.setObjectName(u"closeNotificationBtn")
        icon14 = QIcon()
        icon14.addFile(u":/icons/icons/blueIcons/x-octagon.svg", QSize(), QIcon.Normal, QIcon.Off)
        self.closeNotificationBtn.setIcon(icon14)
        self.closeNotificationBtn.setIconSize(QSize(24, 24))

        self.horizontalLayout_10.addWidget(self.closeNotificationBtn, 0, Qt.AlignRight)


        self.verticalLayout_20.addWidget(self.frame_9)


        self.verticalLayout_19.addWidget(self.popupNotificationSubContainer)


        self.verticalLayout_10.addWidget(self.popupNotificationContainer)

        self.footerContainer = QWidget(self.mainBodyContainer)
        self.footerContainer.setObjectName(u"footerContainer")
        self.horizontalLayout_11 = QHBoxLayout(self.footerContainer)
        self.horizontalLayout_11.setSpacing(0)
        self.horizontalLayout_11.setObjectName(u"horizontalLayout_11")
        self.horizontalLayout_11.setContentsMargins(0, 0, 0, 0)
        self.frame_10 = QFrame(self.footerContainer)
        self.frame_10.setObjectName(u"frame_10")
        self.frame_10.setFrameShape(QFrame.StyledPanel)
        self.frame_10.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_12 = QHBoxLayout(self.frame_10)
        self.horizontalLayout_12.setObjectName(u"horizontalLayout_12")
        self.label_15 = QLabel(self.frame_10)
        self.label_15.setObjectName(u"label_15")
        font6 = QFont()
        font6.setBold(False)
        font6.setItalic(True)
        font6.setWeight(50)
        self.label_15.setFont(font6)

        self.horizontalLayout_12.addWidget(self.label_15)


        self.horizontalLayout_11.addWidget(self.frame_10)

        self.sizeGrip = QFrame(self.footerContainer)
        self.sizeGrip.setObjectName(u"sizeGrip")
        self.sizeGrip.setMinimumSize(QSize(30, 30))
        self.sizeGrip.setMaximumSize(QSize(30, 30))
        self.sizeGrip.setFrameShape(QFrame.StyledPanel)
        self.sizeGrip.setFrameShadow(QFrame.Raised)

        self.horizontalLayout_11.addWidget(self.sizeGrip, 0, Qt.AlignRight)


        self.verticalLayout_10.addWidget(self.footerContainer)


        self.horizontalLayout.addWidget(self.mainBodyContainer)

        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)

        self.centerMenuPages.setCurrentIndex(1)
        self.mainPages.setCurrentIndex(0)
        self.rightMenuPages.setCurrentIndex(1)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
#if QT_CONFIG(tooltip)
        self.menuBtn.setToolTip(QCoreApplication.translate("MainWindow", u"Menu", None))
#endif // QT_CONFIG(tooltip)
        self.menuBtn.setText("")
#if QT_CONFIG(tooltip)
        self.homeBtn.setToolTip(QCoreApplication.translate("MainWindow", u"<html><head/><body><p><span style=\" color:#000000;\">Home</span></p></body></html>", None))
#endif // QT_CONFIG(tooltip)
        self.homeBtn.setText(QCoreApplication.translate("MainWindow", u"Home", None))
#if QT_CONFIG(tooltip)
        self.monitorBtn.setToolTip(QCoreApplication.translate("MainWindow", u"<html><head/><body><p><span style=\" color:#000000;\">CCTV</span></p></body></html>", None))
#endif // QT_CONFIG(tooltip)
        self.monitorBtn.setText(QCoreApplication.translate("MainWindow", u"Monitoring", None))
#if QT_CONFIG(tooltip)
        self.settingsBtn.setToolTip(QCoreApplication.translate("MainWindow", u"<html><head/><body><p><span style=\" color:#ffffff;\">Go to setting</span></p></body></html>", None))
#endif // QT_CONFIG(tooltip)
        self.settingsBtn.setText(QCoreApplication.translate("MainWindow", u"Settings", None))
#if QT_CONFIG(tooltip)
        self.infoBtn.setToolTip(QCoreApplication.translate("MainWindow", u"<html><head/><body><p><span style=\" color:#ffffff;\">Information about the app</span></p></body></html>", None))
#endif // QT_CONFIG(tooltip)
        self.infoBtn.setText(QCoreApplication.translate("MainWindow", u"Information", None))
#if QT_CONFIG(tooltip)
        self.helpBtn.setToolTip(QCoreApplication.translate("MainWindow", u"<html><head/><body><p><span style=\" color:#ffffff;\">Get more help</span></p></body></html>", None))
#endif // QT_CONFIG(tooltip)
        self.helpBtn.setText(QCoreApplication.translate("MainWindow", u"Help", None))
        self.label_2.setText("")
#if QT_CONFIG(tooltip)
        self.closeCenterMenuBtn.setToolTip(QCoreApplication.translate("MainWindow", u"Close Menu", None))
#endif // QT_CONFIG(tooltip)
        self.closeCenterMenuBtn.setText("")
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"Settings", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"GOOGEESE", None))
        self.textEdit.setHtml(QCoreApplication.translate("MainWindow", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-weight:600;\">We do offer</span></p>\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:14pt; font-weight:600;\"><br />for End-users </span></p>\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-weight:600;\">Baby Geese</span> is a <span style=\" font-weight:600;\">charming airport robot</span> designed "
                        "to assist passengers by <span style=\" font-weight:600;\">carrying their luggage</span>, <span style=\" font-weight:600;\">following them</span>, or <span style=\" font-weight:600;\">providing guidance</span>. It also <span style=\" font-weight:600;\">securely collects purchased items</span> on behalf of passengers and <span style=\" font-weight:600;\">reliably fulfills its assigned tasks</span>, offering a <span style=\" font-weight:600;\">trustworthy</span> and <span style=\" font-weight:600;\">delightful airport service</span>.</p>\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p>\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:14pt; font-weight:600;\"><br />for you</span></p>\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; marg"
                        "in-right:0px; -qt-block-indent:0; text-indent:0px;\">Our <span style=\" font-weight:600;\">Googeese AI robot system</span> automates services such as <span style=\" font-weight:600;\">navigation assistance</span>, <span style=\" font-weight:600;\">luggage transport</span>, and <span style=\" font-weight:600;\">storage</span> within airports, maximizing passenger <span style=\" font-weight:600;\">convenience</span>. With an <span style=\" font-weight:600;\">intuitive control interface</span> and <span style=\" font-weight:600;\">efficient management</span>, it enhances the <span style=\" font-weight:600;\">quality</span> of airport services and significantly improves passenger <span style=\" font-weight:600;\">satisfaction</span>, offering an <span style=\" font-weight:600;\">innovative solution</span> for modern airports.</p>\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p>\n"
"<p al"
                        "ign=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Ver. 102.24.001</p></body></html>", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"Help", None))
        self.textEdit_2.setHtml(QCoreApplication.translate("MainWindow", u"<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>", None))
        self.label_6.setText("")
#if QT_CONFIG(tooltip)
        self.label_7.setToolTip(QCoreApplication.translate("MainWindow", u"<html><head/><body><p>Goose Geese, Googeese</p></body></html>", None))
#endif // QT_CONFIG(tooltip)
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"GOOGEESE", None))
#if QT_CONFIG(tooltip)
        self.notificationBtn.setToolTip(QCoreApplication.translate("MainWindow", u"<html><head/><body><p><span style=\" color:#000000;\">Notification list</span></p></body></html>", None))
#endif // QT_CONFIG(tooltip)
        self.notificationBtn.setText("")
#if QT_CONFIG(tooltip)
        self.moreMenuBtn.setToolTip(QCoreApplication.translate("MainWindow", u"<html><head/><body><p><span style=\" color:#000000;\">More</span></p></body></html>", None))
#endif // QT_CONFIG(tooltip)
        self.moreMenuBtn.setText("")
#if QT_CONFIG(tooltip)
        self.profileMenuBtn.setToolTip(QCoreApplication.translate("MainWindow", u"<html><head/><body><p><span style=\" color:#000000;\">Profile</span></p></body></html>", None))
#endif // QT_CONFIG(tooltip)
        self.profileMenuBtn.setText("")
#if QT_CONFIG(tooltip)
        self.minimizeBtn.setToolTip(QCoreApplication.translate("MainWindow", u"<html><head/><body><p><span style=\" color:#000000;\">Minimize Window</span></p></body></html>", None))
#endif // QT_CONFIG(tooltip)
        self.minimizeBtn.setText("")
#if QT_CONFIG(tooltip)
        self.restoreBtn.setToolTip(QCoreApplication.translate("MainWindow", u"<html><head/><body><p><span style=\" color:#000000;\">Restore Window</span></p></body></html>", None))
#endif // QT_CONFIG(tooltip)
        self.restoreBtn.setText("")
#if QT_CONFIG(tooltip)
        self.closeBtn.setToolTip(QCoreApplication.translate("MainWindow", u"Close Window", None))
#endif // QT_CONFIG(tooltip)
        self.closeBtn.setText("")
        self.label_12.setText(QCoreApplication.translate("MainWindow", u"TextLabel", None))
        self.label_30.setText(QCoreApplication.translate("MainWindow", u"Baby Goose 1", None))
        self.label_status_b1.setText("")
        self.label_21.setText(QCoreApplication.translate("MainWindow", u"Motor", None))
        self.label_mt_b1.setText("")
        self.label_23.setText(QCoreApplication.translate("MainWindow", u"LiDAR", None))
        self.label_li_b1.setText("")
        self.label_mode_state_b1.setText("")
        self.label_34.setText(QCoreApplication.translate("MainWindow", u"Baby Goose 2", None))
        self.label_status_b2.setText("")
        self.label_26.setText(QCoreApplication.translate("MainWindow", u"Motor", None))
        self.label_mt_b2.setText("")
        self.label_28.setText(QCoreApplication.translate("MainWindow", u"LiDAR", None))
        self.label_li_b2.setText("")
        self.label_mode_state_b2.setText("")
        self.label_22.setText(QCoreApplication.translate("MainWindow", u"Baby Goose 3", None))
        self.label_status_b3.setText("")
        self.label_27.setText(QCoreApplication.translate("MainWindow", u"Motor", None))
        self.label_mt_b3.setText("")
        self.label_29.setText(QCoreApplication.translate("MainWindow", u"LiDAR", None))
        self.label_li_b3.setText("")
        self.label_mode_state_b3.setText("")
        self.label_25.setText(QCoreApplication.translate("MainWindow", u"Baby Goose 4", None))
        self.label_status_b4.setText("")
        self.label_31.setText(QCoreApplication.translate("MainWindow", u"Motor", None))
        self.label_mt_b4.setText("")
        self.label_32.setText(QCoreApplication.translate("MainWindow", u"LiDAR", None))
        self.label_li_b4.setText("")
        self.label_mode_state_b4.setText("")
        self.videoLabel_TopLeft.setStyleSheet(QCoreApplication.translate("MainWindow", u"background-color: #D3D3D3;", None))
        self.videoLabel_TopLeft.setText(QCoreApplication.translate("MainWindow", u"Baby Goose1 Video_front", None))
        self.videoLabel_TopRight.setStyleSheet(QCoreApplication.translate("MainWindow", u"background-color: #D3D3D3;", None))
        self.videoLabel_TopRight.setText(QCoreApplication.translate("MainWindow", u"Baby Goose1 Video_rear", None))
        self.videoLabel_BottomLeft.setStyleSheet(QCoreApplication.translate("MainWindow", u"background-color: #D3D3D3;", None))
        self.videoLabel_BottomLeft.setText(QCoreApplication.translate("MainWindow", u"Baby Goose2 Video_front", None))
        self.videoLabel_BottomRight.setStyleSheet(QCoreApplication.translate("MainWindow", u"background-color: #D3D3D3;", None))
        self.videoLabel_BottomRight.setText(QCoreApplication.translate("MainWindow", u"Baby Goose2 Video_rear", None))
        self.label_16.setText(QCoreApplication.translate("MainWindow", u"Baby Goose2", None))
        self.label_18.setText(QCoreApplication.translate("MainWindow", u"Emergency Stop", None))
#if QT_CONFIG(tooltip)
        self.emergencyBtn_b1.setToolTip(QCoreApplication.translate("MainWindow", u"Manual Moving Stop", None))
#endif // QT_CONFIG(tooltip)
        self.emergencyBtn_b1.setText("")
        self.label_11.setText(QCoreApplication.translate("MainWindow", u"Baby Goose1", None))
        self.label_10.setText(QCoreApplication.translate("MainWindow", u"Emergency Stop", None))
        self.emergencyBtn_b2.setText("")
        self.label.setText("")
#if QT_CONFIG(tooltip)
        self.closeRightMenuBtn.setToolTip(QCoreApplication.translate("MainWindow", u"<html><head/><body><p><span style=\" color:#000000;\">Close Menu</span></p></body></html>", None))
#endif // QT_CONFIG(tooltip)
        self.closeRightMenuBtn.setText("")
        self.label_8.setText(QCoreApplication.translate("MainWindow", u"Profile", None))
        self.label_9.setText(QCoreApplication.translate("MainWindow", u"More...", None))
        self.label_14.setText(QCoreApplication.translate("MainWindow", u"Notification", None))
        self.label_13.setText(QCoreApplication.translate("MainWindow", u"Notification Messge", None))
#if QT_CONFIG(tooltip)
        self.closeNotificationBtn.setToolTip(QCoreApplication.translate("MainWindow", u"Close Notification", None))
#endif // QT_CONFIG(tooltip)
        self.closeNotificationBtn.setText("")
        self.label_15.setText(QCoreApplication.translate("MainWindow", u"Copyright by Googeese", None))
    # retranslateUi

