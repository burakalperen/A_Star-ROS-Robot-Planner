# A Star Algorithm 

Purpose of these scripts is implementation of A Star algoritm in occupancy grid map.
For now, tested on binary map image. 


* **buidMap.py**:         It converts map image to grid map. If there is obstacle in grid cell, it returns True otherwise returns False.
* **astar.py**  :         Determines appropriate path according to goal world coordinates.
* **transformations.py**: Useful functions for transformations between world and map coordinates
* **node.py**   :         It consists of class that represents world points.



## AStar 

* Mainde başlangıç ve hedef dünya koordinatları Node sınıfının objesi olarak gönderiliyor. 
* Ardından bu node objeleri PathPlanner sınıfına veriliyor. 
* PathPlanner sınıfının constructorında grid map oluşturuluyor.
* Ardından mainde sınıfa ait plan metodu çağırılıyor.
* a_star fonksiyonu ile path oluşturuluyor.
* Anlamak için Astar Function başlığına bakabilirsiniz. bu fonksiyon aşağıda da açıklandığı gibi final değişkenini döndürüyor.
* Final değişkeni construct_path fonksiyonuna gidiyor.



* buildMap: Grid map toplamda her sütun için bir listeye sahip. Bu listelerde grid mapi temsil eden resmin yüksekliği kadar eleman var. Toplamda bu listelerden resmin genişliği kadar var. Yani grid mapteki liste sayısı resmin genişliği kadar. Her listede resmin yüksekliği kadar eleman var. Bu elemanlar True veya False olabilir. True o pixelde engel yok, False ise obstacle demek.



### Astar Function

Öncelikle a_star fonksiyonu  başlangıç noktasını ve hedef noktayı alıyor. 
Opened adında bir liste tanımlıyor ve bu listeyi heap olarak oluşturuyor.
Normalde heap veri tiplerine eleman pushlarken(heappush) eleman en sona ekleniyor. Ama belli bir priority vermek istersen tuple verebilir ve tupledaki ilk elemanın değerine göre sıralayabiliyor.
Yukarıdaki sebepten ötürü elemanlar tuple olarak opened listesine pushlanıyor.

İlk başta start elemanını heap'e pushluyoruz.

Ardından o koordinata yakın 4 farklı move değeri tanımlanmış, bu move değerleri içerisinde for döngüsü oluşturuyoruz.
Ama döngüye girmeden heappop ile opened'dan ilk elemanı yani f değeri en düşük olan elamanı seçiyoruz. f değeri hedefe yakınlığını euclidean distance metoduyla hesaplanan bir değer.
Opened listesi bu f değerine göre sıralanıyor yani priority belirleniyor diyebilirim.

bizim q değerimiz opened'dan aldığımız değer ve her döngüde q değerine move değerini ekleyip, node sınıfı yardımıyla bu yeni noktanın valid olup olmadığını kontrol ediyoruz.
eğer nokta valid ise bu noktanın son olarak hedef noktalara eklenip eklenmeyeceğini kararlaştırıyoruz.

Şöyle düşünelim:
her move döngüsünde 1 tane q noktasına göre 4 farklı move eklenerek 4 farklı nokta oluşturuluyor. Ve her nokta valid mi diye kontrol ediliyor.
q noktası her döngü sonunda heap den en öncelikli nokta olarak seçiliyor. Bu döngüdeki 4 noktadan opened listesine eklenceklere potential_open ve potential_closed değişkenleri yardımıyla karar veriyoruz.
potential_open değişkeni, o anki valid noktanın opened'daki diğer noktalara olan benzerliğini ve f değerinin o değerlerde yüksek olup olmadığını kontrol eder. eğer false ise uygundur.
potential_close değişkeni, aynı mantığı close listesine eklenen diğer noktalara kıyaslayarak yapar.
eğer nokta ikisinde de false alırsa opened listesine eklenir. Ama yine tuple olarak eklenir ve f değerine göre opened listesinde sırasını bulması sağlanır.

Yukarıda bir move döngüsünü anlattık. Döngü bittikten sonra q değeri closed listesine append ile eklenir. sonuçta q değeri en başta opened'da olanlardan f değerine göre en iyisiydi.
Yani q değeri move döngüsüde başka noktalar bulmak için kullanıldı ve döngü bittikten sonra closed'a eklendi.
Zaten en başta q değeri heap'den pop ile alındığı için opened da artık yer almıyor.
Döngü bu şekilde devam ediyor,  closed x,y olarak final noktalarını tutuyor.
Opened ise yüksek ihtimale sahip noktaları tutuyor.

Yukarı move döngüsüyle valid olduğu ve opened'a eklenip eklenmeyeceğine karar verilen noktalar kodda next_node değişkenine karşılık geliyor.

Kodda bir if bloğunda o anki next_node'un bitiş noktasına olan uzaklığı ölçülüyor ve eğer tolerans altındaysa bu artık son noktaya varmışız demektir.
Bu nokta da kod final değişkenini next_node'a eşitleyip, döngüyü kırıyor. 

PERFECT:

Her move döngüsündeki q oluşturduğu next_nodelara kendisini parent olarak veriyor. 
Bu sayede o döngüde oluşan next nodelardan biri diğer seferde q olduğu için bütün q lar birbirinin parenti oluyor sadece ilk q'nun parenti olmuyor.Start olarak seçildiği için.
Şöyle bir ihtimal var eğer o q nun oluşturduğu next nodelardan bir eleman değil de daha önceki bir eleman opened'dan seçilirse sıra bozulacak. ?bunu anla
yani final sadece next_node tolerans altına düşerse ona eşit olacak, o next node diğer q ya, o q next_node iken onu oluşturan q'ya bağlı olacak.

### construct_path Function

fonksiyon sadece final değişkenini alıyor ve önce son node'u bir listeye ekliyor ve ardından onun parentini ekliyor böyle sırayla gidiyor.
.parent None olana kadar devam ediyor zaten sadece start olarak seçtiğimiz ilk q noktasının parent'i none diğerlerinin hep var.

