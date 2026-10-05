

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <iostream>
#include <algorithm>
#include <queue>

using namespace cv;
using namespace std;

struct grafoNode{
    Point ponto;
    // 0 = base, 1 = endpoint ou ponto final, 2 = branchPoint ou junções;
    int tipo;

    grafoNode() {}

    grafoNode(Point p, int t){
        ponto = p;
        tipo = t;
    }
};

struct plant{
    grafoNode base;
    vector<grafoNode> nodes;
    vector<pair<grafoNode, grafoNode>> caule;
    vector<pair<grafoNode, grafoNode>> folhas;

    void bfs(const Mat& grafo, Point primeiro, const vector<Point>& branchPoints, int tipo, Mat& visitados) {
        queue<Point> fila;

        fila.push(primeiro);
        visitados.at<uchar>(primeiro.y, primeiro.x) = 255; 

        int dx[] = { 0,  1,  1,  1,  0, -1, -1, -1};
        int dy[] = {-1, -1,  0,  1,  1,  1,  0, -1};

        while(fila.size() > 0) {
            Point frente = fila.front();
            fila.pop();

            for(int k = 0; k < 8; k++) {
                int nx = frente.x + dx[k];
                int ny = frente.y + dy[k];

                if(nx < 0 || ny < 0 || nx >= grafo.cols || ny >= grafo.rows) continue;

                if(grafo.at<uchar>(ny, nx) == 255 && visitados.at<uchar>(ny, nx) == 0){
                    Point vizinho(nx, ny);

                    for(long unsigned int i = 0; i < branchPoints.size(); i++){
                        if(vizinho == branchPoints[i]){
                            if(tipo == 0){
                                folhas.push_back({grafoNode(vizinho, 2), grafoNode(primeiro, 1)});
                                return;
                            }else{
                                caule.push_back({grafoNode(vizinho, 2), grafoNode(primeiro, 2)});
                                visitados.at<uchar>(ny, nx) = 255;
                                bfs(grafo, vizinho, branchPoints, 1, visitados);
                                return;
                            }
                        }
                    }

                    fila.push(vizinho);
                    visitados.at<uchar>(ny, nx) = 255; 
                }
            }
        }
    }

    plant(vector<Point> branchPoints, vector<Point> endPoints, Mat skeleton){
        //Definindo base
        Point b (0, 0);
        for(long unsigned int i = 0; i < endPoints.size(); i++){
            if(endPoints[i].y > b.y){
                b = endPoints[i];
            }
        }
        base = grafoNode(b, 0);

        //Definindo todos os nodes
        nodes.push_back(base);
        for(long unsigned int i = 0; i < endPoints.size(); i++){
            if(endPoints[i] != b)
                nodes.push_back(grafoNode(endPoints[i], 1));        
        }

        for(long unsigned int i = 0; i < branchPoints.size(); i++){
            nodes.push_back(grafoNode(branchPoints[i], 2));
        }

       //Definindo folhas
        for(long unsigned int i = 0; i < endPoints.size(); i++){
            if(endPoints[i] != b){
                Mat visitadosFolha = Mat::zeros(skeleton.size(), CV_8UC1);
                bfs(skeleton, endPoints[i], branchPoints, 0, visitadosFolha);
            }
        }

        //Definindo caule
        Mat visitadosCaule = Mat::zeros(skeleton.size(), CV_8UC1);
        bfs(skeleton, base.ponto, branchPoints, 1, visitadosCaule);
    }

    void desenhar(int x, int y){
        Mat resultado = Mat(y, x, CV_8UC3, Scalar(255, 255, 255));

        for(long unsigned int i = 0; i < caule.size(); i++){
            circle(resultado, caule[i].first.ponto, 4, Scalar(0, 0, 255), FILLED);
            circle(resultado, caule[i].second.ponto, 4, Scalar(0, 0, 255), FILLED);
            line(resultado, caule[i].first.ponto, caule[i].second.ponto, Scalar(0,0,255), 2);
        }

        for(long unsigned int i = 0; i < folhas.size(); i++){
            circle(resultado, folhas[i].second.ponto, 4, Scalar(0, 0, 0), FILLED);
            line(resultado, folhas[i].first.ponto, folhas[i].second.ponto, Scalar(0 ,255, 0), 2);
        }

        circle(resultado, base.ponto, 4, Scalar(255, 0, 0), FILLED);

        imwrite("resultados/grafo.png", resultado);
    }

    double extrair_comprimento_caule(double escala) {
        double comprimento = 0.0;

        for(auto aresta : caule) {
            comprimento += norm(aresta.first.ponto - aresta.second.ponto);
        }

        return comprimento * escala;
    }

    vector<double> extrair_comprimento_folhas(double escala, int limiar){
        vector<double> s;
        s.reserve(folhas.size());

        for(auto folha : folhas){
            s.push_back(escala * (norm(folha.first.ponto - folha.second.ponto) + limiar));
        }

        return s;
    }

    vector<double> extrair_area_folhas(Mat binary, double escala){
        int num_folhas = 0;
        
        //Separando o caule das folhas
        for(auto node : nodes){
            if(node.tipo == 1){
                num_folhas++;
                continue;
            }

            if(node.tipo == 0)
                continue;

            int raio = 15;
            Point p = node.ponto;
            Rect roi(
                max(0, p.x - raio),
                max(0, p.y - raio),
                min(2 * raio + 1, binary.cols - max(0, p.x - raio)),
                min(2 * raio + 1, binary.rows - max(0, p.y - raio))
            );

            Mat regiao = binary(roi);

            Mat elemento = getStructuringElement(MORPH_ELLIPSE, Size(11,11));
            erode(regiao, regiao, elemento);
        }

        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;

        findContours(binary,contours, hierarchy,RETR_EXTERNAL,  CHAIN_APPROX_SIMPLE);
        Mat drawing = Mat::zeros(binary.size(), CV_8UC3);

        sort(contours.begin(), contours.end(),
        [](const vector<Point>& a, const vector<Point>& b){
            return contourArea(a) > contourArea(b);
        });

        //Desenhando cortorno das folhas e calculando area
        vector<double> s;
        int limite = min((int)contours.size(), num_folhas);;
        s.reserve(limite);
        for(int i = 0; i < limite; i++){
            drawContours(drawing, contours, i, Scalar(0, 255, 0), 2);
            s.push_back(contourArea(contours[i]) * escala);
        }

        imwrite("resultados/contornosFolhas.png", drawing);

        return s;
    }
};

Mat ler_imagem(string nome, string pasta){
    //Lendo imagem original
    Mat img = imread(nome);

    if (img.empty()) {
        cout << "Erro ao carregar imagem" << endl;
        return Mat();;
    }

    imwrite(pasta + "original.jpeg", img);

    return img;
}

Mat conversao_cinza(Mat img, string pasta){
    Mat imgFloat;
    img.convertTo(imgFloat, CV_32F);

    vector<Mat> canais;
    split(imgFloat, canais);

    Mat B = canais[0];
    Mat G = canais[1];
    Mat R = canais[2];

    Mat exg = 2 * G - R - B;

    Mat exgNorm;
    normalize(exg, exgNorm, 0, 255, NORM_MINMAX);

    exgNorm.convertTo(exgNorm, CV_8U);

    imwrite(pasta + "exg.png", exgNorm);

    return exgNorm;
}

Mat convesao_binaria(Mat exg, string pasta){   
    Mat binary;

    threshold(exg, binary, 0, 255, THRESH_BINARY | THRESH_OTSU);
    //threshold(exg, binary, 0, 255, THRESH_BINARY | THRESH_TRIANGLE);

    int borderSize = 3;
    // Pinta uma borda preta em volta da imagem para desconectar do limite que tava bugando a identificação da base
    rectangle(binary, Point(0,0), Point(binary.cols-1, binary.rows-1), Scalar(0), borderSize);

    imwrite(pasta + "binary.png", binary);

    return binary;
}

Mat preenchimento_buracos(Mat binary, string pasta) {
    Mat inversa;
    bitwise_not(binary, inversa);

    floodFill(inversa, Point(0, 0), Scalar(0));

    Mat preenchida;
    bitwise_or(binary, inversa, preenchida);

    imwrite(pasta + "preenchida.png", preenchida);

    return preenchida;
}

Mat extracao_esqueleto(Mat binary, string pasta){
    Mat skeleton;
    thinning(binary, skeleton, cv::ximgproc::THINNING_ZHANGSUEN);
    //thinning(binary, skeleton, cv::ximgproc::THINNING_GUOHALL);
    
    imwrite(pasta + "esqueleto.png", skeleton);

    return skeleton;
}

Mat podamento(Mat skeleton, int limiar, string pasta){
    for(int l = 0; l < limiar; l++){
        vector<Point> endPointsRemoviveis;
        // Arrays para percorrer os 8 vizinhos em sentido horário ao redor do pixel central (j, i)
        // Começando de cima (0, -1) e girando...
        int dx[] = { 0,  1,  1,  1,  0, -1, -1, -1};
        int dy[] = {-1, -1,  0,  1,  1,  1,  0, -1};

        for (int i = 1; i < skeleton.rows - 1; i++) {
            for (int j = 1; j < skeleton.cols - 1; j++) {

                // Pula se não for um pixel do esqueleto
                if (skeleton.at<uchar>(i, j) != 255)
                    continue;

                int transicoes = 0;

                // Percorre os 8 vizinhos em círculo
                for (int k = 0; k < 8; k++) {
                    // k é o vizinho atual, next_k é o próximo vizinho no círculo
                    int next_k = (k + 1) % 8;

                    uchar p1 = skeleton.at<uchar>(i + dy[k], j + dx[k]);
                    uchar p2 = skeleton.at<uchar>(i + dy[next_k], j + dx[next_k]);

                    // Conta apenas as transições de Fundo (0) para Esqueleto (255)
                    if (p1 == 0 && p2 == 255) {
                        transicoes++;
                    }
                }

                //Removo caso seja endpoints
                if (transicoes == 1) {
                    endPointsRemoviveis.push_back(Point(j, i));
                }
            }
        }

        for(long unsigned int i = 0; i < endPointsRemoviveis.size(); i++){
            skeleton.at<uchar>(endPointsRemoviveis[i].y, endPointsRemoviveis[i].x) = 0;
        }    
    }
    
    imwrite(pasta + "esqueletoPodado.png", skeleton);

    return skeleton;
}

Mat contagem_branchpoints_endpoints(Mat skeleton, vector<Point> &branchPoints, vector<Point> &endPoints,string pasta){
    Mat resultado;
    cvtColor(skeleton, resultado, COLOR_GRAY2BGR);

    // Arrays para percorrer os 8 vizinhos em sentido horário ao redor do pixel central (j, i)
    // Começando de cima (0, -1) e girando...
    int dx[] = { 0,  1,  1,  1,  0, -1, -1, -1};
    int dy[] = {-1, -1,  0,  1,  1,  1,  0, -1};

    // Contando branchs e end points usando Transições (Crossing Number)
    for (int i = 1; i < skeleton.rows - 1; i++) {
        for (int j = 1; j < skeleton.cols - 1; j++) {

            // Pula se não for um pixel do esqueleto
            if (skeleton.at<uchar>(i, j) != 255)
                continue;

            int transicoes = 0;

            // Percorre os 8 vizinhos em círculo
            for (int k = 0; k < 8; k++) {
                int next_k = (k + 1) % 8;

                uchar p1 = skeleton.at<uchar>(i + dy[k], j + dx[k]);
                uchar p2 = skeleton.at<uchar>(i + dy[next_k], j + dx[next_k]);

                if (p1 == 0 && p2 == 255) {
                    transicoes++;
                }
            }

            // Desenha com base no número de transições
            if (transicoes == 1) {
                // Endpoint (Verde)
                endPoints.push_back(Point(j, i));
                circle(resultado, Point(j, i), 4, Scalar(0, 255, 0), FILLED); 
            }
            else if (transicoes >= 3) {
                // Branch point (Vermelho)
                branchPoints.push_back(Point(j, i));
                circle(resultado, Point(j, i), 4, Scalar(0, 0, 255), FILLED); 
            }
        }
    }

    imwrite(pasta + "branchpoints.png", resultado);

    return resultado;
}

double calcular_escala(Mat regua) {
    string pasta = "regua/";

    Mat exg = conversao_cinza(regua, pasta);

    Mat binary = convesao_binaria(exg, pasta);

    Mat preenchida = preenchimento_buracos(binary, pasta);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(preenchida, contours, hierarchy,
                 RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    sort(contours.begin(), contours.end(),
         [](const vector<Point>& a, const vector<Point>& b) {
             return contourArea(a) > contourArea(b);
         });

    return 84 / arcLength(contours[0], true);
}

pair<Mat, Mat> separar_imagem(Mat original){ 
    vector<vector<Point>> contours; 
    vector<Vec4i> hierarchy; 

    Mat exg = conversao_cinza(original, "pasta");
    Mat binary = convesao_binaria(exg, "pasta");

    findContours(binary, contours, hierarchy,
                  RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    int indiceRegua = -1;
    int indicePlanta = -1;

    double maiorArea = 0;

    for(int i = 0; i < contours.size(); i++){
        Rect retangulo = boundingRect(contours[i]);

        int largura = retangulo.width;
        int altura = retangulo.height;
        int area = largura * altura;

        if(altura > largura * 2 && area > 10000){
            indiceRegua = i;
        }
    }

    for(int i = 0; i < contours.size(); i++){

        if(i == indiceRegua)
            continue;

        Rect retangulo = boundingRect(contours[i]);
        int area = retangulo.width * retangulo.height;

        if(area > maiorArea){
            maiorArea = area;
            indicePlanta = i;
        }
    }

    if(indiceRegua == -1 || indicePlanta == -1){
        return {Mat(), Mat()};
    }

    Rect retanguloRegua = boundingRect(contours[indiceRegua]);
    Rect retanguloPlanta = boundingRect(contours[indicePlanta]);

    Mat regua = original(retanguloRegua).clone();
    Mat planta = original(retanguloPlanta).clone();

    rectangle(original, retanguloRegua, Scalar(0, 0, 255), 3);
    rectangle(original, retanguloPlanta, Scalar(0, 255, 0), 3);

    imwrite("separacao.jpg", original);

    return make_pair(regua, planta);
}

int main() {
    Mat img = ler_imagem("original.jpg", "planta/");

    auto separadas = separar_imagem(img);
    Mat planta = separadas.second;
    imwrite("planta/planta.jpg", planta);

    Mat exg = conversao_cinza(planta, "planta/");

    Mat binary = convesao_binaria(exg, "planta/");

    Mat preenchida = preenchimento_buracos(binary, "planta/");

    Mat skeleton = extracao_esqueleto(preenchida, "planta/");
    
    int limiar = 45;
    skeleton = podamento(skeleton, limiar, "planta/");

    vector<Point> branchPoints;
    vector<Point> endPoints;
    contagem_branchpoints_endpoints(skeleton, branchPoints, endPoints, "planta/");

    //Montando grafo a partir da 
    Mat regua = separadas.first;
    imwrite("regua/regua.jpg", regua);
    double escala = calcular_escala(regua);

    plant p = plant(branchPoints, endPoints, skeleton);
    p.desenhar(img.cols, img.rows);

    vector<double> comp = p.extrair_comprimento_folhas(escala, limiar);

    for(auto i : comp){
        cout << "O comprimento da folha é: " << i << "cm" << endl;
    }

    return 0;
}