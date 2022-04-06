![panorama of field](figs/columbia-all.jpg)

# Esercitazione 4 parte 1 #

Per questa esercitazione utilizzerete i risultati delle prime due 
esercitazioni, più precisamente i file `process_image.cpp`, `filter_image.
cpp` e `resize_image.cpp`. Vengono forniti i file di soluzione già inclusi, 
ma potete sovrascriverli con i vostri. Oltre a questo, troverete i test1 e 
test2 come verifica che tutto funzioni. 

test5 invece è il programma che produrrà la foto panoramica basata sulla 
vostra implementazione dei vari algoritmi necessari, oltre a effettuare 
qualche test. 

Le funzioni di timing possono essere rimosse aggiungendo `#define TIME(a)` 
alla fine di `utils.h`

## Creare un panorama ##

Quesa prima parte dell'esercitazione 4 copre gli algoritmi per estrarre ie 
key points e per fare il matching con quelli di un'altra immagine. 
Nella seconda parte implementerete la trasformazione di un immagine 
nell'altra, creando il panorama.

L'algoritmo di alto livello è già implementato, si trova nel file 
`src/panorama_image.cpp`. La sua struttura è approssimativamente questa:

    Image panorama_image(const Image& a, const Image& b, float sigma, int corner_method, float thresh, int window, int nms, float     inlier_thresh, int iters, int cutoff, float acoeff)
      {
      // Calculate corners and descriptors
      vector<Descriptor> ad = harris_corner_detector(a, sigma, thresh, window, nms, corner_method);
      vector<Descriptor> bd = harris_corner_detector(b, sigma, thresh, window, nms, corner_method);

      // Find matches
      vector<Match> m = match_descriptors(ad, bd);

      // Run RANSAC to find the homography
      Matrix Hba = RANSAC(m, inlier_thresh, iters, cutoff);

      // Stitch the images together with the homography
      return combine_images(a, b, Hba, acoeff);
      }

I corner verranno estratti con un Harris corner detector. Quinidi faremo il 
matching di questi punti. Nella seconda parte dell'esercitazione vedremo gli 
altri passi in dettaglio.

## 0. Visualizzazione con Pangolin ##
Per poter fare debugging introduciamo le funzioni `detect_and_draw_corners` 
e  `find_and_draw_matches` che visualizza i risultati delle funzioni che 
implementerete. Inoltre, per visualizzare le gli output in questa 
esercitazione useremo Pangolin:

![pangolin](figs/screenshot.png)

È un software gratuito che permette di modificare i parametri degli 
algoritmi che implementerete e di visualizzare i risultati immediatamente. 
Per sapernte di più andate sul repo: 
[Pangolin GitHub](https://github.com/stevenlovegrove/Pangolin). 
Pangolin supporta Linux, Windows and MacOS. Di seguito le istruzioni di 
installazione su Linux e gli altri sistemi. Per Linux possiamo aiutarvi, per 
Windows e MacOS dovrete cavarvela con i forum online se qualcosa non va. 

### 0.1 Installazione su Ubuntu o Debian ###
    sudo apt-get install libglew-dev libxkbcommon-dev
    git clone https://github.com/stevenlovegrove/Pangolin.git
    cd Pangolin
    mkdir build
    cd build
    cmake ..
    make -j4

Ovviamente installate tutte le dipendenze che dovessero mancare.

### 0.2 Istallazione su Fedora o CentOS ###
    sudo yum install glew-devel.x86_64
    git clone https://github.com/stevenlovegrove/Pangolin.git
    cd Pangolin
    mkdir build
    cd build
    cmake ..
    make -j4

Installate tutte le dipendenze che dovessero mancare.

### 0.4 Installazione su MacOS (senza supporto) ###
Ecco un link che può servire: 
[Pangolin GitHub macOS issue](https://github.com/stevenlovegrove/Pangolin/issues/298)

### 0.5 Istallazione su Windows ###
Potete trovare la soluzione dell'esercizio nella sottocartella  
`/vs/cse576-hw5` . Vi rimando al repo originale per i dettagli: 
[repo cse576-hw5](https://github.com/holynski/cse576_sp20_hw5)

### 0.6 Compilazione e set-up ###
In Linux dovrebbe bastare compilare per rendere Pangolin diponibile al 
vostro progetto. Fate cmake dell'esercitazione in questo modo:

    cd ai-lab_es4
    cd build
    cmake ..

e dovreste avere questo output:

    ---------------------------------------------
    Pangolin found!!!!
    Building WITH Visualization
    ---------------------------------------------

### 0.7 Shortcuts utili ###
Lo zoom si controlla con lo scroll. Una volta aumentato lo zoom di molto, 
pigiate `n` per cambiare tra interpolazione NN e bilineare.
Il carattere `a` normalizza l'intensità della patch visualizzata tra 0 e 1. 
In questo modo potete analizzare meglio zone sotto o sovra esposte. 
Se invece premete `b` nella linea di comando dovreste vedere il valore 
min/max nella regione evidenziata.

## 1. Harris corner detection ##

L'algoritmo fondamentale è questo:

    Calcolo delle derivate Ix e Iy
    Calcolo delle componenti IxIX IyIy e IxIy
    Calcolo della matrice di struttura come somma pesata delle misure adiacenti.
    Calcolo dei corner usando la funzione R = det(S)/tr(S), oppure estraendo 
        il 2° autovalore
    Utilizzare non-max suppression per ottenere i corner

## 1.1 Calcolo della matrice di struttura ##

Completate la funzione `Image structure_matrix(const Image& im2, float sigma)`
 in `harris_image.cpp`. Questa effettua i primi tre step dell'algoritmo: 
calcolo delle derivate, calcolo dei fattori e somma pesata delle derivate 
adiacenti. Questa somma pesata può ottenrsi facilmente con un blur Gaussiano.
Per i gradienti usate i filtir di sobel, come avete fatto nell'esercitazione 
3 sul Canny Edge detector. 

### 1.1b Make a fast smoother ###

Per rendere l'implementazione più efficiente,
usate la proprietà di separabilità del filtro gaussiano e implementatelo
usando `make_gx_filter` e `make_gy_filter` dell'esercitazione 2.
Invece di usare un filtro NxN bidimensionale, ne userete due 1xN e Nx1.

Per farlo completate `Image make_1d_gaussian(float sigma)` e 
`Image smooth_image(const Image& im, float sigma)` .

## 1.2 Computer cornerness from structure matrix ##

Fill in `Image cornerness_response(const Image& S, int method)`. Return `det(S)/tr(S)` for each pixel, if `method==0`, or if `method==1` return exact 2nd eigenvalue. The case of `method==1` is optional.

## 1.3 Non-maximum suppression ##

We only want local maximum responses to our corner detector so that the matching is easier. Fill in `Image nms_image(const Image& im, int w)`.

For every pixel in `im`, check every neighbor within `w` pixels (Chebyshev distance). Equivalently, check the `2w+1` window centered at each pixel. If any responses are stronger, suppress that pixel's response (set it to a very low negative number).

## 1.4 Complete the Harris detector ##

Fill in the missing sections of `vector<Descriptor> detect_corners(const Image& im, const Image& nms, float thresh, int window)`. The function should return an vector of descriptors for corners in the image. Code for calculating the descriptors is provided, though you can vary the size of the described window around each corner with the parameter `window`.

After you complete this function you should be able to calculate corners and descriptors for an image! Try running:

    Image im = load_image("data/Rainier1.png");
    Image corners=detect_and_draw_corners(im, 2, 0.2, 5, 3, 0);
    save_image(corners, "output/corners");


This will detect corners using a Gaussian window of 2 sigma, a "cornerness" threshold of 100, and an nms distance of 3 (or window of 7x7). It should give you something like this:

![rainier corners](figs/corners.jpg)

Corners are marked with the crosses. They seem pretty sensible! Lots of corners near where snow meets rock and such. Try playing with the different values to see how the affect our corner detector.

## 2 Patch matching ##

To get a panorama we have to match up the corner detections with their appropriate counterpart in the other image. The descriptor code is already written for you. It consists of nearby pixels except with the center pixel value subtracted. This gives us some small amount of invariance to lighting conditions. Note that the function `Descriptor describe_index(const Image& im, int x, int y, int w)` also take as a parameter the window for the size of the descriptor.

The rest of the homework takes place in `src/panorama_image.cpp`.

## 2.1 Distance metric ##
For comparing patches we'll use L1 distance. Squared error (L2 distance) can be problematic with outliers as we saw in class. We don't want a few rogue pixels to throw off our matching function. L1 distance (sum absolute difference) is better behaved with some outliers.

Implement float `l1_distance(float *a, float *b, int n)` between two vectors of floats. The vectors and how many values they contain is passed in.

## 2.2a Find the best matches from A to B ##

First we'll look through descriptors for `Image a` and find their best match with descriptors from `Image b`. Fill in `vector<int> match_descriptors_a2b(const vector<Descriptor>& a, const vector<Descriptor>& b)`.

## 2.2b Eliminate non-symmetric matches  ##

`match_descriptors_a2b` finds the best match in `b` for each descriptor in `a`. What if that descriptor in `b` itself has a better match in `a`. Be a good matchmaker and find the matches between the descriptors in `a[]` and `b[]`, such that each descriptor in a match is the best for other.

Once this is done we can show the matches we discover between the images:

    Image a = load_image("data/Rainier1.png");
    Image b = load_image("data/Rainier2.png");
    Image m = find_and_draw_matches(a, b, 2, 0.4, 7, 3, 0);
    save_image(m, "output/matches");

Which gives you:

![matches](figs/matches.jpg)
