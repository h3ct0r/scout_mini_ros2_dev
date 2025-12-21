# Centralized UI styles and color scheme for ros2_unbag

PRIMARY = "#3b82f6"  # lighter blue
PRIMARY_HOVER = "#2563eb"
PRIMARY_ACTIVE = "#1d4ed8"
TEXT_PRIMARY = "#0f172a"
TEXT_SECONDARY = "#475569"
BORDER = "#cfd6e3"
BG_MAIN = "#edf1f7"
BG_CARD = "#f7f9fc"
BG_WHITE = "#ffffff"
GRAY_SOFT = "#f1f3f7"
SELECTION = "#e5edff"
CHECKED_BG = "#e9fcdc"
BADGE_SELECTED_BG = "#ecfdf3"
BADGE_SELECTED_BORDER = "#3dc522"
BADGE_SELECTED_TEXT = "#1F6516"
BADGE_UNSELECTED_BG = "#f8fafc"
BADGE_UNSELECTED_BORDER = "#d1d5db"
BADGE_UNSELECTED_TEXT = "#6b7280"

TOP_BAR_STYLE = (
    f"#topBar {{ background-color: {BG_MAIN}; border-bottom: 1px solid {BORDER}; }}"
    f"#topBar QLabel#title {{ color: {TEXT_PRIMARY}; font-size: 20px; font-weight: bold; font-family: 'Ubuntu', 'Ubuntu Bold', 'Ubuntu Medium', monospace; }}"
    f"QPushButton#headerLoadButton {{ background-color: {PRIMARY}; color: {BG_WHITE}; border-radius: 18px; padding: 10px 18px; font-weight: 600; font-size: 14px; }}"
    f"QPushButton#headerLoadButton:hover {{ background-color: {PRIMARY_HOVER}; }}"
    f"QPushButton#headerLoadButton:pressed {{ background-color: {PRIMARY_ACTIVE}; }}"
)

LEFT_CONTAINER_STYLE = (
    f"#leftContainer {{ background-color: {BG_CARD}; border: 1px solid {BORDER}; border-radius: 6px; }}"
    f"#leftContainer QGroupBox {{ font-weight: 600; font-size: 13px; color: {TEXT_PRIMARY}; }}"
    "#leftContainer QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 4px 0 4px 2px; }"
)

SCROLL_STYLE = (
    f"QScrollArea {{ border: 1px solid {BORDER}; border-radius: 6px; background: {BG_WHITE}; }}"
    f"QScrollArea > QWidget > QWidget {{ background: {BG_WHITE}; }}"
)

GLOBAL_CONTAINER_STYLE = (
    f"#globalContainer {{ background-color: {BG_CARD}; border: 1px solid {BORDER}; border-radius: 6px; }}"
    f"#globalContainer QGroupBox {{ font-weight: 600; font-size: 13px; color: {TEXT_PRIMARY}; }}"
    "#globalContainer QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 4px 0 4px 2px; }"
)

TOPIC_LIST_STYLE = (
    f"QTreeWidget {{ background: {BG_WHITE}; border: 1px solid {BORDER}; border-radius: 6px; color: {TEXT_PRIMARY}; }}"
    "QTreeWidget::item { height: 28px; }"
    f"QTreeWidget::item:selected {{ background: {SELECTION}; color: {TEXT_PRIMARY}; }}"
    f"QHeaderView::section {{ background: {BG_MAIN}; border: none; padding: 6px 8px; font-weight: bold; color: {TEXT_PRIMARY}; }}"
)

EXPORT_BUTTON_STYLE = (
    f"QPushButton {{ background-color: {PRIMARY}; color: {BG_WHITE}; border: none; border-radius: 15px; padding: 12px 18px; font-weight: 700; font-size: 15px; }}"
    f"QPushButton:hover {{ background-color: {PRIMARY_HOVER}; }}"
    f"QPushButton:pressed {{ background-color: {PRIMARY_ACTIVE}; }}"
    f"QPushButton:disabled {{ background-color: {BORDER}; color: {TEXT_SECONDARY}; }}"
)

# Text styles
LEFT_HEADER_STYLE = "font-size: 16px; font-weight: 700; margin-bottom: 6px; color: #0f172a;"
TS_HEADER_STYLE = "font-size: 16px; font-weight: 700; margin-bottom: 4px; color: #0f172a;"
TS_TOPIC_STYLE = "font-size: 15px; font-weight: 600; margin-bottom: 12px; color: #1f2937;"
EXPORT_BADGE_SELECTED_STYLE = (
    f"border: 2px solid {BADGE_SELECTED_BORDER}; color: {BADGE_SELECTED_TEXT}; background: {BADGE_SELECTED_BG}; "
    "border-radius: 20px; padding: 6px; margin: 0; font-size: 25px; font-weight: 700;"
)
EXPORT_BADGE_UNSELECTED_STYLE = (
    f"border: 2px solid {BADGE_UNSELECTED_BORDER}; color: {BADGE_UNSELECTED_TEXT}; background: {BADGE_UNSELECTED_BG}; "
    "border-radius: 20px; padding: 6px; margin: 0; font-size: 25px; font-weight: 700;"
)
HELP_TEXT_STYLE = "color: gray; font-style: italic; margin-top: 20px;"
EMPTY_HINT_STYLE = "color: gray; font-style: italic; font-size: 18px"
INPUT_ERROR_STYLE = "QLineEdit[error='true'] { border: 1px solid #ef4444; background: #fff1f2; }"

# Progress and feedback styles
PROGRESS_BAR_STYLE = (
    f"QProgressBar {{ background: {BG_CARD}; border: 1px solid {BORDER};"
    " border-radius: 6px; padding: 2px; text-align: center; color: #1f2937; }"
    f" QProgressBar::chunk {{ background-color: {PRIMARY}; border-radius: 4px; }}"
)

SUCCESS_BANNER_STYLE = (
    f"#feedbackBanner {{ background: {BADGE_SELECTED_BG}; border: 1px solid {BADGE_SELECTED_BORDER};"
    f" border-radius: 8px; padding: 8px 10px; }} "
    f"#feedbackBanner QLabel {{ color: {BADGE_SELECTED_TEXT}; font-weight: 600; }}"
)
