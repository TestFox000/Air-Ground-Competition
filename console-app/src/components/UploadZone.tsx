import { useCallback, useRef, useState } from "react";
import { useNavigate } from "react-router";
import { Loader2 } from "lucide-react";
import { toast } from "sonner";
import { IconUpload } from "./Icons";
import { trpc } from "@/providers/trpc";
import { MAX_VIDEO_BYTES, MAX_VIDEO_LABEL } from "@contracts/pipeline";
import { cn } from "@/lib/utils";

type Phase = "idle" | "reading" | "uploading";

const PHASE_TEXT: Record<Exclude<Phase, "idle">, string> = {
  reading: "读取视频元数据…",
  uploading: "上传至本地存储…",
};

function readVideoMeta(file: File): Promise<{ durationMs: number; width: number; height: number }> {
  return new Promise((resolve, reject) => {
    const url = URL.createObjectURL(file);
    const v = document.createElement("video");
    v.preload = "metadata";
    v.onloadedmetadata = () => {
      URL.revokeObjectURL(url);
      resolve({
        durationMs: Math.round((v.duration || 0) * 1000),
        width: v.videoWidth || 0,
        height: v.videoHeight || 0,
      });
    };
    v.onerror = () => {
      URL.revokeObjectURL(url);
      reject(new Error("无法解析该视频文件，请确认封装格式（推荐 MP4/H.264）"));
    };
    v.src = url;
  });
}

function toBase64(file: File): Promise<string> {
  return new Promise((resolve, reject) => {
    const reader = new FileReader();
    reader.onload = () => {
      const result = String(reader.result);
      resolve(result.slice(result.indexOf(",") + 1));
    };
    reader.onerror = () => reject(new Error("文件读取失败"));
    reader.readAsDataURL(file);
  });
}

export function UploadZone({ compact }: { compact?: boolean }) {
  const navigate = useNavigate();
  const inputRef = useRef<HTMLInputElement>(null);
  const [phase, setPhase] = useState<Phase>("idle");
  const [dragOver, setDragOver] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const utils = trpc.useUtils();
  const upload = trpc.videos.upload.useMutation();

  const busy = phase !== "idle";

  const handleFile = useCallback(
    async (file: File | undefined | null) => {
      if (!file || busy) return;
      setError(null);
      if (!file.type.startsWith("video/")) {
        setError("仅支持视频文件（MP4 / MOV / WebM 等）");
        return;
      }
      if (file.size > MAX_VIDEO_BYTES) {
        setError(`文件 ${(file.size / 1024 / 1024).toFixed(1)}MB，超过 ${MAX_VIDEO_LABEL} 上限（演示链路建议 30 秒内的短片段）`);
        return;
      }
      try {
        setPhase("reading");
        const meta = await readVideoMeta(file);
        if (meta.durationMs <= 0) throw new Error("视频时长为 0，文件可能损坏");
        setPhase("uploading");
        const contentBase64 = await toBase64(file);
        const res = await upload.mutateAsync({
          name: file.name,
          contentBase64,
          contentType: file.type || "video/mp4",
          ...meta,
        });
        await utils.videos.list.invalidate();
        toast.success("视频已入库，可以启动识别-规划链");
        navigate(`/videos/${res.id}`);
      } catch (err) {
        const msg = err instanceof Error ? err.message : "上传失败";
        setError(msg);
        toast.error(msg);
        setPhase("idle");
      }
    },
    [busy, upload, navigate, utils],
  );

  return (
    <div>
      <input
        ref={inputRef}
        type="file"
        accept="video/*"
        className="hidden"
        onChange={(e) => {
          void handleFile(e.target.files?.[0]);
          e.target.value = "";
        }}
      />
      <button
        type="button"
        disabled={busy}
        onClick={() => inputRef.current?.click()}
        onDragOver={(e) => {
          e.preventDefault();
          setDragOver(true);
        }}
        onDragLeave={() => setDragOver(false)}
        onDrop={(e) => {
          e.preventDefault();
          setDragOver(false);
          void handleFile(e.dataTransfer.files?.[0]);
        }}
        className={cn(
          "group flex w-full cursor-pointer flex-col items-center justify-center gap-3 border border-dashed text-center transition-colors duration-150",
          compact ? "min-h-[132px] px-4 py-5" : "min-h-[220px] px-6 py-10",
          dragOver ? "border-hud bg-hud/10" : "border-hairline bg-panel/40 hover:border-hud/50 hover:bg-panel",
          busy && "cursor-wait opacity-80",
        )}
        aria-label="上传航拍视频"
      >
        {busy ? (
          <Loader2 className="h-7 w-7 animate-stage-spin text-hud" />
        ) : (
          <IconUpload
            className={cn(
              "text-hud transition-transform duration-150 group-hover:-translate-y-0.5",
              compact ? "text-[26px]" : "text-[36px]",
            )}
            style={compact ? { width: 26, height: 26 } : { width: 38, height: 38 }}
          />
        )}
        <div className="space-y-1">
          <p className="text-[14px] font-medium text-foreground">
            {busy ? PHASE_TEXT[phase] : "拖入航拍视频，或点击选择文件"}
          </p>
          <p className="font-data text-[11px] tracking-wider text-muted-foreground">
            {busy ? "PLEASE WAIT" : `MP4 / MOV / WebM · ≤ ${MAX_VIDEO_LABEL} · 数据仅保存在本系统`}
          </p>
        </div>
      </button>
      {error && (
        <p className="mt-2 border border-redsig/40 bg-redsig/10 px-3 py-2 font-data text-[12px] text-redsig" role="alert">
          {error}
        </p>
      )}
    </div>
  );
}
