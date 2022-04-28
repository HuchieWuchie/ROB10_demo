import os
import json
import shutil

if __name__ == '__main__':
    path = "/home/daniel/Transformations/Transformations"
    path_parsed = "/home/daniel/Transformations/transformations_parsed"
    root, dirs, _ = next(os.walk(path))
    index = 0
    for dir in dirs:
        sub_root, sub_dirs, _ = next(os.walk(os.path.join(root,dir)))
        for sub_dir in sub_dirs:
            _, _, files = next(os.walk(os.path.join(sub_root, sub_dir)))
            try:
                index_annotation = files.index("annotation.json")
                index_transformation = files.index("transformation.npy")
                f = open(os.path.join(sub_root, sub_dir, files[index_annotation]))
                data = json.load(f)
                transformation_class = data['objects'][0]['name']
                #if transformation_class == 'Ladle' or transformation_class == 'Scissors' or transformation_class == 'scissor':
                    #print(os.path.join(sub_root, sub_dir, files[index_annotation]))

                if not os.path.isdir(os.path.join(path_parsed, transformation_class)):
                    os.mkdir(os.path.join(path_parsed, transformation_class))

                src = os.path.join(sub_root, sub_dir, files[index_transformation])
                dst = os.path.join(path_parsed, transformation_class, files[index_transformation] + str(index))
                shutil.copy(src, dst)
                index = index + 1
                #print(os.path.join(sub_root, sub_dir, files[index_transformation]))
            except ValueError:
                continue
            #json = open(os.path.join(sub_root, sub_dir, files[3]))
