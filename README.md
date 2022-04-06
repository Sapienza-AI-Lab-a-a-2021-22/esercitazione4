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
In order to help debugging we introduce the functions `detect_and_draw_corners` and `find_and_draw_matches` which visualize the results of the functions that you implemented. Look in `test2.cpp` and in the code as to how to use them.

Also if you want to bring your visualization to the next level we introduce another tool: Pangolin!
![pangolin](figs/screenshot.png)

It allows you to modify the parameters of your algorithms and visualize the results immediately. To make use of it go to [Pangolin GitHub](https://github.com/stevenlovegrove/Pangolin). Pangolin supports Linux, Windows and MacOS. However, we can only help you install in on Linux, but if you manage to build it on MacOS or Windows please share!

### 0.1 Install Pangolin on Ubuntu or Debian ###
    sudo apt-get install libglew-dev libxkbcommon-dev
    git clone https://github.com/stevenlovegrove/Pangolin.git
    cd Pangolin
    mkdir build
    cd build
    cmake ..
    make -j4

Install any missing dependencies.

### 0.2 Install Pangolin on Fedora or CentOS (Including CSE VM) ###
    sudo yum install glew-devel.x86_64
    git clone https://github.com/stevenlovegrove/Pangolin.git
    cd Pangolin
    mkdir build
    cd build
    cmake ..
    make -j4

Install any missing dependencies.


### 0.3 Install Pangolin on CSE machines without root access ###
    git clone https://github.com/stevenlovegrove/Pangolin.git
    cd Pangolin
    mkdir build
    cd build

    #### CHANGE NEXT LINE
    hw2dir=/home/swetko/cse576/cse457-hw5-2020 #### PUT YOUR cse576-hw5-2020 folder here!!!
    #### CHANGE PREVIOUS LINE

    cmake .. -DGLEW_INCLUDE_DIR=$hw2dir/glew/include/ -DGLEW_LIBRARY=$hw2dir/glew/lib/libGLEW.so -DGLEW_FOUND=TRUE
    make -j4

### 0.4 Install Pangolin on MacOS ###
Here's a link that might help: [Pangolin GitHub macOS issue](https://github.com/stevenlovegrove/Pangolin/issues/298)

### 0.5 Install Pangolin on Windows ###
You can find Visual Studio Solution for this homework in the `/vs/cse576-hw5` subfolder. It includes the pangolin library itself in `/vs`.
You might have to retarget each project if you are using a different version of VS or whatever. VS will suggest the proper version automatically. The problem is that we are supplying Pangolin compiled with a specific version of VS and your project has to match it.

IMPORTANT: Once you compile the visualization tools, try running them. If you get just a black window, try resizing the window. That should make the GUI appear.

### 0.6 Set up your project ###
On linux once you compile Pangolin, it should be found by our project.

    cd cse455-hw2
    cd build
    cmake ..


You should see this output if Pangolin is found

    ---------------------------------------------
    Pangolin found!!!!
    Building WITH Visualization
    ---------------------------------------------

### 0.7 Useful shortcuts ###
You can use scroll to zoom in/out in pictures. Try zooming in A LOT and pressing 'n'. That switches between NN and Bilinear display of the images in pangolin. You can use right click to move the image around and left click to select regions. Try selecting a region and pressing 'a'. Try different regions! Do you notice what's happening? It normalizes the intensity so the patch you selected is between 0 and 1. This way you can inspect underexposed pictures, etc. Try pressing 'b' in a region! Look in the command line! It shows the min/max value in region!

## 1. Harris corner detection ##

We'll be implementing Harris corner detection as discussed in class. The basic algorithm is:

    Calculate image derivatives Ix and Iy.
    Calculate measures IxIx, IyIy, and IxIy.
    Calculate structure matrix components as weighted sum of nearby measures.
    Calculate Harris "cornerness" as estimate of 2nd eigenvalue: det(S)/tr(S)
              Alternatively (Optionally) find the exact 2nd eigenvalue
    Run non-max suppression on response map

## 1.1 Compute the structure matrix ##

Fill in `Image structure_matrix(const Image& im2, float sigma)` in `harris_image.cpp`. This will perform the first 3 steps of the algorithm: calculating derivatives, the corresponding measures, and the weighted sum of nearby derivative information. As discussed in class, this weighted sum can be easily computed with a Gaussian blur. For gradients use the sobel filters (slightly smoothed versions of the regular -101 filter). You should use your `make_gx_filter` and `make_gy_filter` from HW2.

### 1.1b Make a fast smoother ###

You want a fast corner detector! You have to decompose the Gaussian blur from one large 2d convolution to 2 1d convolutions. Instead of using an N x N filter you should convolve with a 1 x N filter followed by the same filter flipped to be N x 1.

Fill in `Image make_1d_gaussian(float sigma)` and `Image smooth_image(const Image& im, float sigma)` to use this decomposed Gaussian smoothing.

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
