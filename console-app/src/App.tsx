import { Routes, Route } from "react-router";
import { Toaster } from "sonner";
import Home from "./pages/Home";
import VideoDetail from "./pages/VideoDetail";
import Docs from "./pages/Docs";
import NotFound from "./pages/NotFound";

export default function App() {
  return (
    <>
      <Routes>
        <Route path="/" element={<Home />} />
        <Route path="/videos/:id" element={<VideoDetail />} />
        <Route path="/docs" element={<Docs />} />
        <Route path="*" element={<NotFound />} />
      </Routes>
      <Toaster theme="dark" position="bottom-right" toastOptions={{ style: { borderRadius: 0 } }} />
    </>
  );
}
