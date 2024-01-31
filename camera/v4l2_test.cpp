// Reference : https://github.com/wbadry/Simple-OpenCV-CPP-VSCode/tree/master
/*
v4l2-ctl --list-formats
ioctl: VIDIOC_ENUM_FMT
    Type: Video Capture

    [0]: 'MJPG' (Motion-JPEG, compressed)
    [1]: 'YUYV' (YUYV 4:2:2)
*/
// Reference :
// https://stackoverflow.com/questions/69240481/capturing-yuyv-in-c-using-v4l2
// sudo apt-get install libv4l-dev

// Reference :
// https://medium.com/@athul929/capture-an-image-using-v4l2-api-5b6022d79e1d
// *************************************************************************************
/* v4l2-ctl -d /dev/video3 --list-formats-ext
Résultat obtenu avec le cable d'origine (USB3.2) :
ioctl: VIDIOC_ENUM_FMT
    Type: Video Capture

    [0]: 'UYVY' (UYVY 4:2:2)
        Size: Discrete 1280x720
            Interval: Discrete 0.008s (120.000 fps)
            Interval: Discrete 0.017s (60.000 fps)
        Size: Discrete 1920x1080
            Interval: Discrete 0.017s (60.000 fps)
        Size: Discrete 1920x1200
            Interval: Discrete 0.018s (55.000 fps)
    [1]: 'MJPG' (Motion-JPEG, compressed)
        Size: Discrete 1280x720
            Interval: Discrete 0.008s (120.000 fps)
            Interval: Discrete 0.017s (60.000 fps)
        Size: Discrete 1920x1080
            Interval: Discrete 0.008s (120.000 fps)
            Interval: Discrete 0.017s (60.000 fps)
            Interval: Discrete 0.033s (30.000 fps)
        Size: Discrete 1920x1200
            Interval: Discrete 0.009s (114.000 fps)
            Interval: Discrete 0.017s (60.000 fps)
Exemple de code :
int fd = open("/dev/video0", O_RDWR);
int set_format(int fd) {
    struct v4l2_format format = {0};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = 320;
    format.fmt.pix.height = 240;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    format.fmt.pix.field = V4L2_FIELD_NONE;
    int res = ioctl(fd, VIDIOC_S_FMT, &format);
    if(res == -1) {
        perror("Could not set format");
        exit(1);
    }
    return res;
}
*/
// *************************************************************************************
/* Controle de la camera :
v4l2-ctl --list-ctrls --device /dev/video0

User Controls

                     brightness 0x00980900 (int)    : min=-15 max=15 step=1 default=0 value=0
                       contrast 0x00980901 (int)    : min=0 max=30 step=1 default=9 value=9
                     saturation 0x00980902 (int)    : min=0 max=60 step=1 default=16 value=16
        white_balance_automatic 0x0098090c (bool)   : default=1 value=1
                          gamma 0x00980910 (int)    : min=40 max=500 step=1 default=220 value=220
                           gain 0x00980913 (int)    : min=1 max=40 step=1 default=1 value=1
           power_line_frequency 0x00980918 (menu)   : min=0 max=2 default=0 value=0 (Disabled)
      white_balance_temperature 0x0098091a (int)    : min=1000 max=10000 step=50 default=4500 value=4500 flags=inactive
                      sharpness 0x0098091b (int)    : min=0 max=127 step=1 default=16 value=16

Camera Controls

                  auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=0 value=0 (Auto Mode)
         exposure_time_absolute 0x009a0902 (int)    : min=0 max=10000 step=1 default=312 value=312 flags=inactive

Set white balance
v4l2-ctl -d /dev/video3 -c white_balance_temperature_auto=0
v4l2-ctl -d /dev/video3 -c white_balance_temperature=3000

Tell everything about camera
v4l2-ctl -d /dev/video3 --all
*/
// *************************************************************************************
/* Capturer une image (autre source d'info : https://trac.gateworks.com/wiki/linux/v4l2) :
v4l2-ctl --device /dev/video3 --set-fmt-video=width=1920,height=1200,pixelformat=MJPG --stream-mmap --stream-skip=1 --stream-to=frame1920x1200.jpg --stream-count=1
v4l2-ctl --device /dev/video3 --set-fmt-video=width=1920,height=1080,pixelformat=UYVY --stream-mmap --stream-to=frame1920x1080.uyvy --stream-count=1
convert -size 1920x1080 -depth 16 uyvy:frame1920x1080.uyvy frame1920x1080.png
v4l2-ctl --device /dev/video3 --set-fmt-video=width=1920,height=1200,pixelformat=UYVY --stream-mmap --stream-to=frame1920x1200.uyvy --stream-count=1
convert -size 1920x1200 -depth 16 uyvy:frame1920x1200.uyvy frame1920x1200.png
*/
/* Récupérer des bouts de code de https://github.com/gjasny/v4l-utils */
// convert -size 1920x1200 -depth 16 uyvy:image000_1920x1200x16in422.uyvy image000_1920x1200x16in422.png

#include <errno.h>
#include <fcntl.h>
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;

#define WIDTH 1920            // La largeur de l'image à capturer
#define HEIGHT 1200           // La hauteur de l'image à capturer
#define FPS 55
#define NB_FRAMES 1
#define BUFFER_COUNT 4  // Le nombre de tampons à demander
#define FILEOUTNAME "0capture/image%03d_1920x1200x16in422.uyvy"

// Une structure pour stocker les informations d'un tampon
struct buffer {
    void *start;    // Le pointeur vers le début du tampon
    size_t length;  // La longueur du tampon en octets
};

// Une fonction pour afficher un message d'erreur et quitter le programme
void error_exit(const char *message) {
    perror(message);
    exit(EXIT_FAILURE);
}

// Une fonction pour vérifier le résultat d'un appel système et afficher un message d'erreur si nécessaire
void check_result(int result, const char *message) {
    if (result == -1) {
        error_exit(message);
    }
}

const char* find_See3CAM_24CUG() {
    std::ifstream devices("/proc/bus/input/devices");

    if (!devices.is_open()) {
        std::cerr << "Erreur lors de l'ouverture du fichier des périphériques." << std::endl;
        return "";
    }

    std::string line;
    std::string deviceName;
    bool isUVCDevice = false;

    while (std::getline(devices, line)) {
        if (line.find("UVC Video") != std::string::npos) {
            isUVCDevice = true;
        } else if (line.find("Handlers") != std::string::npos && isUVCDevice) {
            size_t pos = line.find("event");
            if (pos != std::string::npos) {
                size_t eventNumStart = pos + 5;
                size_t eventNumEnd = line.find(" ", eventNumStart);
                std::string eventNumStr = line.substr(eventNumStart, eventNumEnd - eventNumStart);
                int eventNum = std::stoi(eventNumStr);

                std::cout << "Périphérique V4L2 UVC trouvé : /dev/video" << eventNum << std::endl;
            }

            isUVCDevice = false;
        }
    }

    devices.close();
    return "";
}

// Une fonction pour ouvrir le périphérique de capture et retourner son descripteur de fichier
int open_device(const char* device_name) {
    int fd = open(device_name, O_RDWR);  // Ouvrir le périphérique en mode lecture-écriture
    check_result(fd, "open");       // Vérifier si l'ouverture a réussi

    printf("Périphérique : %s\n", device_name);
    return fd;
}

// Une fonction pour fermer le périphérique de capture
void close_device(int fd) {
    int result = close(fd);         // Fermer le descripteur de fichier
    check_result(result, "close");  // Vérifier si la fermeture a réussi
}

// Une fonction pour interroger les capacités du périphérique de capture
void query_capability(int fd) {
    struct v4l2_capability
        cap;                                        // Créer une structure pour stocker les capacités
    int result = ioctl(fd, VIDIOC_QUERYCAP, &cap);  // Appeler la commande VIDIOC_QUERYCAP
    check_result(result, "ioctl");                  // Vérifier si la commande a réussi

    // Vérifier si le périphérique est un périphérique de capture vidéo
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        error_exit("Le périphérique n'est pas un périphérique de capture vidéo");
    }

    // Vérifier si le périphérique supporte le streaming I/O
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        error_exit("Le périphérique ne supporte pas le streaming I/O");
    }

    // Afficher les informations du périphérique
    printf("Pilote : %s\n", cap.driver);
    printf("Carte : %s\n", cap.card);
    printf("Bus : %s\n", cap.bus_info);
    printf("Version : %u.%u.%u\n", (cap.version >> 16) & 0xFF, (cap.version >> 8) & 0xFF, cap.version & 0xFF);
    printf("Capacités : %08x\n", cap.capabilities);
}

// Une fonction pour définir le format d'image du périphérique de capture
void set_format(int fd) {
    struct v4l2_format fmt;                       // Créer une structure pour stocker le format
    memset(&fmt, 0, sizeof(fmt));                 // Initialiser la structure à zéro
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;       // Spécifier le type de tampon
    fmt.fmt.pix.width = WIDTH;                    // Spécifier la largeur de l'image
    fmt.fmt.pix.height = HEIGHT;                  // Spécifier la hauteur de l'image
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;  // Spécifier le format de pixel
    fmt.fmt.pix.field = V4L2_FIELD_NONE;          // Spécifier le mode de balayage
    int result = ioctl(fd, VIDIOC_S_FMT, &fmt);   // Appeler la commande VIDIOC_S_FMT
    check_result(result, "ioctl");                // Vérifier si la commande a réussi

    struct v4l2_streamparm parm;
    memset(&parm, 0, sizeof(parm));
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    result = ioctl(fd, VIDIOC_G_PARM, &parm);
    check_result(result, "ioctl");  // Vérifier si la commande a réussi

    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = static_cast<uint32_t>(FPS * parm.parm.capture.timeperframe.numerator);
    result = ioctl(fd, VIDIOC_S_PARM, &parm);
    check_result(result, "ioctl");  // Vérifier si la commande a réussi

    // Afficher le format d'image
    printf("Format d'image :\n");
    printf("Largeur : %u\n", fmt.fmt.pix.width);
    printf("Hauteur : %u\n", fmt.fmt.pix.height);
    printf("Format de pixel : %c%c%c%c\n",
           fmt.fmt.pix.pixelformat & 0xFF,
           (fmt.fmt.pix.pixelformat >> 8) & 0xFF,
           (fmt.fmt.pix.pixelformat >> 16) & 0xFF,
           (fmt.fmt.pix.pixelformat >> 24) & 0xFF);
    printf("Taille de l'image : %u\n", fmt.fmt.pix.sizeimage);
}

// Une fonction pour demander un tampon au périphérique de capture
struct buffer *request_buffer(int fd) {
    struct v4l2_requestbuffers
        req;                                       // Créer une structure pour stocker la requête
    memset(&req, 0, sizeof(req));                  // Initialiser la structure à zéro
    req.count = BUFFER_COUNT;                      // Spécifier le nombre de tampons à demander
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;        // Spécifier le type de tampon
    req.memory = V4L2_MEMORY_MMAP;                 // Spécifier le mode de mappage mémoire
    int result = ioctl(fd, VIDIOC_REQBUFS, &req);  // Appeler la commande VIDIOC_REQBUFS
    check_result(result, "ioctl");                 // Vérifier si la commande a réussi

    // Vérifier si le nombre de tampons alloués est suffisant
    if (req.count < BUFFER_COUNT) {
        error_exit("Le nombre de tampons alloués est insuffisant");
    }

    // Allouer de la mémoire pour stocker les informations des tampons
    struct buffer *buffers = (struct buffer *)calloc(req.count, sizeof(struct buffer));
    if (!buffers) {
        error_exit("Erreur d'allocation de mémoire");
    }

    // Récupérer les informations de chaque tampon
    for (int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf;                     // Créer une structure pour stocker les informations du tampon
        memset(&buf, 0, sizeof(buf));               // Initialiser la structure à zéro
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;     // Spécifier le type de tampon
        buf.memory = V4L2_MEMORY_MMAP;              // Spécifier le mode de mappage mémoire
        buf.index = i;                              // Spécifier l'index du tampon
        result = ioctl(fd, VIDIOC_QUERYBUF, &buf);  // Appeler la commande VIDIOC_QUERYBUF
        check_result(result, "ioctl");              // Vérifier si la commande a réussi

        // Stocker les informations du tampon dans le tableau
        buffers[i].length = buf.length;                                                                        // Stocker la longueur du tampon
        buffers[i].start = v4l2_mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);  // Faire le mappage mémoire du tampon
        if (buffers[i].start == MAP_FAILED) {
            error_exit("Erreur de mappage mémoire");
        }
    }

    return buffers;
}

// Une fonction pour démarrer le flux vidéo du périphérique de capture
void start_streaming(int fd, struct buffer *buffers) {
    // Mettre en file d'attente tous les tampons
    for (int i = 0; i < BUFFER_COUNT; i++) {
        struct v4l2_buffer buf;                     // Créer une structure pour stocker les informations du tampon
        memset(&buf, 0, sizeof(buf));               // Initialiser la structure à zéro
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;     // Spécifier le type de tampon
        buf.memory = V4L2_MEMORY_MMAP;              // Spécifier le mode de mappage mémoire
        buf.index = i;                              // Spécifier l'index du tampon
        int result = ioctl(fd, VIDIOC_QBUF, &buf);  // Appeler la commande VIDIOC_QBUF
        check_result(result, "ioctl");              // Vérifier si la commande a réussi
    }

    // Démarrer le flux vidéo
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;          // Spécifier le type de tampon
    int result = ioctl(fd, VIDIOC_STREAMON, &type);  // Appeler la commande VIDIOC_STREAMON
    check_result(result, "ioctl");                   // Vérifier si la commande a réussi
}

// Une fonction pour capturer une image du périphérique de capture
// return 0 if it is an empty frame
unsigned int capture_image(int fd, struct buffer *buffers, unsigned int frame_number) {
    // Créer une structure pour stocker les informations du tampon
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));            // Initialiser la structure à zéro
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;  // Spécifier le type de tampon
    buf.memory = V4L2_MEMORY_MMAP;           // Spécifier le mode de mappage mémoire

    // Retirer le tampon de la file d'attente
    int result = ioctl(fd, VIDIOC_DQBUF, &buf);  // Appeler la commande VIDIOC_DQBUF
    check_result(result, "ioctl");               // Vérifier si la commande a réussi

    // Traiter l'image capturée
    /*
    printf("Buff index     : %i\n", buf.index);
    printf("Buff length    : %u\n", buf.length);
    printf("Buff bytesused : %u\n", buf.bytesused);
    */

    // Check if "empty frame" or not
    unsigned int bytesused = buf.bytesused;
    if (bytesused != 0) {
        char out_name[256];
        sprintf(out_name, FILEOUTNAME, frame_number);
        FILE *fout = fopen(out_name, "w");
        if (!fout) {
            perror("Cannot write image");
            exit(EXIT_FAILURE);
        }
        fwrite(buffers[buf.index].start, bytesused, 1, fout);
        fflush(fout);
        fclose(fout);

        // Créer un objet Mat UYVY à partir du buffer binaire
        Mat uyvy_image(HEIGHT, WIDTH, CV_8UC2, buffers[buf.index].start);

        // Convertir l'image UYVY en RGB
        Mat rgb_image;
        cvtColor(uyvy_image, rgb_image, COLOR_YUV2BGR_UYVY);

        // Convertir l'image en HSV (Teinte, Saturation, Valeur)
        cv::Mat hsvImage;
        cv::cvtColor(rgb_image, hsvImage, cv::COLOR_BGR2HSV);

        // Détecter une plage de couleur (cf https://www.peko-step.com/en/tool/hsvrgb_en.html avec H(0-360) à diviser par 2 ! et S,V (0-255))
        // Cf aussi : https://learnopencv.com/color-spaces-in-opencv-cpp-python/
        // Cf aussi : https://theailearner.com/tag/cv2-inrange-opencv-python/
        // Cf aussi : https://stackoverflow.com/questions/48109650/how-to-detect-two-different-colors-using-cv2-inrange-in-python-opencv
        // Définir les plages de couleurs pour la couleur souhaitée
        cv::Scalar lowerRed = cv::Scalar(101, 240, 200);  // bleu
        cv::Scalar upperRed = cv::Scalar(110, 255, 255);

        // Filtrer l'image pour ne conserver que les pixels souhaités
        cv::Mat mask;
        cv::inRange(hsvImage, lowerRed, upperRed, mask);

        // Trouver les contours des objets souhaités
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Dessiner les contours sur l'image originale
        cv::drawContours(rgb_image, contours, -1, cv::Scalar(0, 255, 0), 2);

        // Afficher l'image RGB
        namedWindow("Image UYVY", WINDOW_NORMAL);
        imshow("Image UYVY", rgb_image);
        waitKey(0);
    }

    // Remettre le tampon en file d'attente
    result = ioctl(fd, VIDIOC_QBUF, &buf);  // Appeler la commande VIDIOC_QBUF
    check_result(result, "ioctl");          // Vérifier si la commande a réussi

    return bytesused;
}

// Une fonction pour arrêter le flux vidéo du périphérique de capture
void stop_streaming(int fd) {
    // Arrêter le flux vidéo
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;           // Spécifier le type de tampon
    int result = ioctl(fd, VIDIOC_STREAMOFF, &type);  // Appeler la commande VIDIOC_STREAMOFF
    check_result(result, "ioctl");                    // Vérifier si la commande a réussi
}

// Une fonction pour libérer le tampon du périphérique de capture
void release_buffer(struct buffer *buffers) {
    for (int i = 0; i < BUFFER_COUNT; i++) {
        int result =
            munmap(buffers[i].start, buffers[i].length);  // Libérer le mappage mémoire du tampon
        check_result(result, "munmap");                   // Vérifier si la libération a réussi
    }
    free(buffers);  // Libérer la mémoire allouée pour le tableau
}

// Une fonction principale pour tester le code
int main() {

    const char* device_name = find_See3CAM_24CUG();

    // Ouvrir le périphérique de capture
    int fd = open_device(device_name);

    // Interroger les capacités du périphérique
    query_capability(fd);

    // Définir le format d'image du périphérique
    set_format(fd);

    // Demander un tampon au périphérique
    struct buffer *buffers = request_buffer(fd);

    // Démarrer le flux vidéo du périphérique
    start_streaming(fd, buffers);

    // Capturer des images du périphérique
    bool chrono_start = false;
    std::chrono::_V2::system_clock::time_point start_time, end_time;
    for (unsigned int frame_number = 0; frame_number < NB_FRAMES;
         frame_number++) {
        unsigned int frame_empty;
        do {
            frame_empty = capture_image(fd, buffers, frame_number);
        } while (frame_empty == 0);
        if (!chrono_start) {
            start_time = std::chrono::high_resolution_clock::now();
            chrono_start = true;
        }
    }
    end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    printf("Time to capture %u frames : %lld milliseconds\n", NB_FRAMES, static_cast<long long>(duration.count()));

    // Arrêter le flux vidéo du périphérique
    stop_streaming(fd);

    // Libérer le tampon du périphérique
    release_buffer(buffers);

    // Fermer le périphérique de capture
    close_device(fd);

    return 0;
}
