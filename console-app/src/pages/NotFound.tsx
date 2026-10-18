import { Link } from "react-router";
import { TopBar } from "@/components/TopBar";
import { IconDroneSignal } from "@/components/Icons";

export default function NotFound() {
  return (
    <div className="min-h-dvh">
      <TopBar />
      <main className="mx-auto flex max-w-[1100px] flex-col items-center px-4 py-24 text-center sm:px-6">
        <IconDroneSignal className="animate-standby text-hud/60 hud-glow" style={{ width: 72, height: 72 }} />
        <p className="font-data hud-glow mt-6 text-[42px] font-bold tracking-[0.2em] text-hud">404</p>
        <p className="mt-2 text-[14px] text-muted-foreground">信号丢失：该页面不在任务范围内</p>
        <Link to="/" className="btn-hud mt-8">返回控制台</Link>
      </main>
    </div>
  );
}
