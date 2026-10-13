from pathlib import Path
import shutil, sys
src=Path(__file__).resolve().parents[1]/'model_source'
dest=Path(sys.argv[1]) if len(sys.argv)>1 else src.parent/'build_source'
shutil.copytree(src,dest,dirs_exist_ok=True)
p=dest/'mainwindow.cpp'; text=p.read_text(encoding='utf8')
old='QString dbPath = "E:/Desk/department/shuziluansheng/shuziluansheng/xjp/daima/Yangtze_Mile235_270.db";'
new='QString dbPath = qEnvironmentVariable("YINGONGZHOU_CHART_DB");\n    if (dbPath.isEmpty()) dbPath = QCoreApplication::applicationDirPath() + "/Yangtze_Mile235_270.db";'
if old not in text: raise RuntimeError('Expected chart path not found; inspect source before building')
p.write_text(text.replace(old,new,1),encoding='utf8')
print(dest)
