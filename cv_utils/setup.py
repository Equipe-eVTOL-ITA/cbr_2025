from setuptools import setup, find_packages

package_name = 'cbr_2025_cv_utils'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    install_requires=['setuptools', 'ultralytics'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 package for computer vision utilities.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_classifier = yolo_classifier.yolo_classifier:main',
            'barcode = barcode_detector.oak_bar:main',
            'qrcode = qrcode_detector.qrcode_detector:main',
            'base_detector = base_detector.base_detector:main',
            'target_base_detector = target_detector.base_detector:main',
            'target_package_detector = target_detector.package_detector:main',
            'window_detector = window_detector.window_detector:main',
            'qr_code_detector = qrcode_detector.qrcode_detector:main'
        ],
    }
)
