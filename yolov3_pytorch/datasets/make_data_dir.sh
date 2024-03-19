rm -r train valid

echo "Extract compressed annotation file"
unzip -q task_our_data-2024_01_19_07_44_41-yolo_1.1.zip
rm obj.data train.txt
mv obj.names tstl.txt

echo "Remove image.txt which has no annotation"
find obj_train_data -type f -size 0 -exec rm {} \;

echo "Split annotations into train set and valid set"
mkdir -p train/Annotations valid/Annotations
ls obj_train_data | shuf --random-source=<(yes 42) -n $(( $(ls obj_train_data | wc -l) * 80 / 100 )) | xargs -I{} mv obj_train_data/{} train/Annotations
mv obj_train_data/* valid/Annotations && rm -rf obj_train_data

mkdir -p train/ImageSets valid/ImageSets
ls train/Annotations | sed 's/\.txt$//' > train/ImageSets/all.txt
ls valid/Annotations | sed 's/\.txt$//' > valid/ImageSets/all.txt



echo "Extarct compressed image file"
unzip -q dataset.zip

echo "Split images into train set and valid set"
mkdir -p train/JPEGImages valid/JPEGImages
xargs -a train/ImageSets/all.txt -I{} mv dataset/{}.png train/JPEGImages/{}.png
xargs -a valid/ImageSets/all.txt -I{} mv dataset/{}.png valid/JPEGImages/{}.png
rm -r dataset
